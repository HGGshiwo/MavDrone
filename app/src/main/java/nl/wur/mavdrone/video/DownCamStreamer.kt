package nl.wur.mavdrone.video

import java.io.ByteArrayOutputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.SurfaceTexture
import android.graphics.YuvImage
import android.view.Surface

import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.camera.CameraStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager

import io.reactivex.rxjava3.core.Observable
import io.reactivex.rxjava3.disposables.Disposable
import io.reactivex.rxjava3.schedulers.Schedulers

import nl.wur.mavdrone.utils.Logger

/**
 * Publishes the live camera image as an MJPEG HTTP stream so that external machines
 * (e.g. the ROS ground station) can pull it and republish it as an image topic.
 *
 * Uses the shared ICameraStreamManager decode (the same one the FPV widget feeds on)
 * with NV21 output, throttled and JPEG encoded. The stream is served as
 * multipart/x-mixed-replace on [port], path is ignored.
 */
class DownCamStreamer(
    private val port: Int,
    private val fpsLimit: Int = 15,
    private val jpegQuality: Int = 60,
    private val targetWidth: Int = 480,
    private val targetHeight: Int = 320
) {
    private val log = Logger("DownCamStreamer")
    private val running = AtomicBoolean(false)
    private val invalidLayoutLogged = AtomicBoolean(false)
    private val listenerRegistered = AtomicBoolean(false)
    private var offscreenSurface: Surface? = null
    private var offscreenTexture: SurfaceTexture? = null

    // Offscreen decode-driving surface size
    private val offscreenSurfaceWidth = 640
    private val offscreenSurfaceHeight = 480
    private var serverSocket: ServerSocket? = null
    private var serverThread: Thread? = null
    private var connectionDisposable: Disposable? = null
    private val clients = mutableListOf<Socket>()
    private val frameLock = Object()

    @Volatile
    private var latestJpeg: ByteArray? = null

    @Volatile
    private var frameSeq = 0L

    @Volatile
    private var lastEncodeMillis = 0L

    private val frameListener =
        ICameraStreamManager.CameraFrameListener { data, p1, p2, p3, p4, format ->
            onFrame(data, p1, p2, p3, p4, format)
        }

    fun start() {
        if (!running.compareAndSet(false, true)) {
            return
        }

        serverThread = Thread {
            try {
                serverSocket = ServerSocket(port)
                log.i("DownCam MJPEG server listening on port $port")
                while (running.get()) {
                    val socket = serverSocket!!.accept()
                    Thread { serveClient(socket) }.apply { name = "DownCamClient" }.start()
                }
            } catch (e: Exception) {
                if (running.get()) {
                    log.w("DownCam MJPEG server stopped: ${e.message}")
                }
            }
        }.apply {
            name = "DownCamServer"
            start()
        }

        // Registration is NOT gated on the product connection key: KeyConnection is
        // unreliable on this aircraft (see README), and addFrameListener is harmless
        // before the stream exists. Retry until the SDK accepts it. An offscreen
        // surface is attached as well because some firmwares only decode when at
        // least one surface is registered.
        connectionDisposable = Observable.interval(0, 2, TimeUnit.SECONDS)
            .subscribeOn(Schedulers.io())
            .takeWhile { running.get() }
            .subscribe(
                {
                    if (listenerRegistered.getAndSet(true)) {
                        return@subscribe
                    }
                    try {
                        // Keep decoding even when no app widget is attached
                        CameraStreamManager.getInstance().setKeepAliveDecoding(true)

                        // The SDK keeps at most one frame listener per channel across
                        // service restarts; clear any stale one before adding ours
                        try {
                            CameraStreamManager.getInstance().removeFrameListener(frameListener)
                        } catch (_: Exception) {
                        }

                        CameraStreamManager.getInstance().addFrameListener(
                            ComponentIndexType.LEFT_OR_MAIN,
                            ICameraStreamManager.FrameFormat.NV21,
                            frameListener
                        )
                        attachOffscreenSurface()
                        log.i("DownCam frame listener registered")
                    } catch (e: Exception) {
                        listenerRegistered.set(false)
                        if (e.message?.contains("already") == true) {
                            // Our listener from a previous service round is still in
                            // place and receiving frames; that is the state we want
                            attachOffscreenSurface()
                            log.i("DownCam frame listener already registered, reusing it")
                        } else {
                            log.w("DownCam frame listener registration failed, retrying: ${e.message}")
                        }
                    }
                },
                { error ->
                    if (running.get()) {
                        log.w("DownCam registration loop failed: ${error.message}")
                    }
                }
            )
    }

    fun stop() {
        if (!running.compareAndSet(true, false)) {
            return
        }

        connectionDisposable?.dispose()
        connectionDisposable = null
        listenerRegistered.set(false)

        try {
            CameraStreamManager.getInstance().removeFrameListener(frameListener)
        } catch (_: Exception) {
        }
        detachOffscreenSurface()

        try {
            serverSocket?.close()
        } catch (_: Exception) {
        }
        serverThread?.interrupt()

        synchronized(clients) {
            clients.forEach { client ->
                try {
                    client.close()
                } catch (_: Exception) {
                }
            }
            clients.clear()
        }
        log.i("DownCam MJPEG server stopped")
    }

    private fun onFrame(
        data: ByteArray,
        p1: Int,
        p2: Int,
        p3: Int,
        p4: Int,
        format: ICameraStreamManager.FrameFormat
    ) {
        if (!running.get()) {
            return
        }

        // Measured argument order of the 5 ints is (offset, size, width, height);
        // the documented order (width, height, offset, stride) does not match the
        // values seen in the field. Validate against the buffer size and fall back
        // to the documented order if the measured one does not fit.
        var width = p3
        var height = p4
        var offset = p1
        if (width <= 0 || height <= 0 || offset < 0 || width * height * 3 / 2 > data.size) {
            if (p1 > 0 && p2 > 0 && p1 * p2 * 3 / 2 <= data.size) {
                width = p1
                height = p2
                offset = 0
            } else {
                return
            }
        }
        val stride = width

        val now = System.currentTimeMillis()
        if (now - lastEncodeMillis < 1000 / fpsLimit) {
            return
        }
        lastEncodeMillis = now

        try {
            val nv21 = packNv21(data, width, height, offset, stride, format)
            if (nv21 == null) {
                if (invalidLayoutLogged.compareAndSet(false, true)) {
                    log.w(
                        "DownCam frame layout not decodable: ${width}x$height offset=$offset " +
                                "stride=$stride format=$format data=${data.size} bytes " +
                                "(needed ${width * height * 3 / 2}); frames will be skipped"
                    )
                }
                return
            }

            // Downscale before publishing: keeps the WiFi stream at ~0.3 MB/s and the
            // encode cost low on the phone
            val jpeg = encodeJpeg(nv21, width, height)
            if (jpeg != null) {
                synchronized(frameLock) {
                    latestJpeg = jpeg
                    frameSeq++
                    frameLock.notifyAll()
                }
            }
        } catch (e: Exception) {
            log.w("DownCam frame encode failed: ${e.message}")
        }
    }

    // NV21 -> Bitmap -> scale to the target size -> JPEG. Skips scaling when the frame
    // already fits; decode/scale allocations are bounded by the fps limit.
    private fun encodeJpeg(nv21: ByteArray, width: Int, height: Int): ByteArray? {
        val yuv = YuvImage(nv21, ImageFormat.NV21, width, height, null)
        val fullOut = ByteArrayOutputStream()
        yuv.compressToJpeg(Rect(0, 0, width, height), jpegQuality, fullOut)
        val fullJpeg = fullOut.toByteArray()

        if (width <= targetWidth && height <= targetHeight) {
            return fullJpeg
        }

        val fullBitmap = BitmapFactory.decodeByteArray(fullJpeg, 0, fullJpeg.size)
            ?: return fullJpeg

        val scale = maxOf(
            targetWidth.toFloat() / width,
            targetHeight.toFloat() / height
        )
        val scaledWidth = (width * scale).toInt().coerceAtLeast(2)
        val scaledHeight = (height * scale).toInt().coerceAtLeast(2)

        val smallBitmap = Bitmap.createScaledBitmap(fullBitmap, scaledWidth, scaledHeight, true)
        val smallOut = ByteArrayOutputStream()
        smallBitmap.compress(Bitmap.CompressFormat.JPEG, jpegQuality, smallOut)

        smallBitmap.recycle()
        fullBitmap.recycle()
        return smallOut.toByteArray()
    }

    // Decoder buffers come in more than one layout depending on the stream: some are
    // contiguous standard NV21 (size = w*h*3/2) with bogus offset/stride, others carry
    // row padding. Detect the layout from the actual buffer size instead of trusting
    // the parameters; returns null when no layout fits (frame is then skipped).
    private fun packNv21(
        data: ByteArray,
        width: Int,
        height: Int,
        offset: Int,
        stride: Int,
        format: ICameraStreamManager.FrameFormat
    ): ByteArray? {
        if (width <= 0 || height <= 0) {
            return null
        }

        val packed = ByteArray(width * height * 3 / 2)
        val uvRows = height / 2

        // 1. Buffer is exactly a standard NV21 frame: copy wholesale, the reported
        //    offset/stride do not apply to this buffer
        if (offset == 0 && data.size == packed.size) {
            System.arraycopy(data, 0, packed, 0, packed.size)
            return packed
        }

        // 2. Stride layout: Y rows and UV rows both padded to stride, UV plane after
        //    the Y plane; validate every access against the real buffer size
        val uvStart = offset + height * stride
        if (stride >= width && uvStart + (uvRows - 1) * stride + width <= data.size) {
            for (row in 0 until height) {
                System.arraycopy(data, offset + row * stride, packed, row * width, width)
            }
            for (row in 0 until uvRows) {
                System.arraycopy(
                    data, uvStart + row * stride,
                    packed, width * height + row * width, width
                )
            }
            return packed
        }

        // 3. Semiplanar fallback: UV plane directly after w*h Y bytes (no padding)
        if (offset + width * height + packed.size - width * height <= data.size) {
            System.arraycopy(data, offset, packed, 0, width * height)
            System.arraycopy(
                data, offset + width * height, packed, width * height,
                packed.size - width * height
            )
            return packed
        }

        return null
    }

    // Some firmwares only run the decoder when at least one surface is registered;
    // attach a small offscreen surface so decoding works without the app UI
    private fun attachOffscreenSurface() {
        if (offscreenSurface != null) {
            return
        }
        val texture = SurfaceTexture(false)
        texture.setDefaultBufferSize(offscreenSurfaceWidth, offscreenSurfaceHeight)
        val surface = Surface(texture)
        CameraStreamManager.getInstance().putCameraStreamSurface(
            ComponentIndexType.LEFT_OR_MAIN,
            surface,
            offscreenSurfaceWidth,
            offscreenSurfaceHeight,
            ICameraStreamManager.ScaleType.CENTER_INSIDE
        )
        offscreenSurface = surface
        offscreenTexture = texture
    }

    private fun detachOffscreenSurface() {
        offscreenSurface?.let {
            try {
                CameraStreamManager.getInstance().removeCameraStreamSurface(it)
                it.release()
            } catch (_: Exception) {
            }
        }
        offscreenSurface = null
        offscreenTexture?.release()
        offscreenTexture = null
    }

    private fun serveClient(socket: Socket) {
        val requestLine = readHttpRequestLine(socket)

        // GET /snapshot returns a single JPEG frame, useful for isolating HTTP issues
        // from frame-pipeline issues; everything else gets the MJPEG stream
        if (requestLine.contains("/snapshot")) {
            log.i("DownCam snapshot requested")
            try {
                val output = socket.getOutputStream()
                val jpeg = synchronized(frameLock) { latestJpeg }
                if (jpeg == null) {
                    output.write(
                        ("HTTP/1.0 503 Service Unavailable\r\n" +
                                "Content-Type: text/plain\r\n" +
                                "Content-Length: 24\r\n\r\n" +
                                "no frame available yet\r\n").toByteArray()
                    )
                } else {
                    output.write(
                        ("HTTP/1.0 200 OK\r\n" +
                                "Content-Type: image/jpeg\r\n" +
                                "Content-Length: ${jpeg.size}\r\n" +
                                "Connection: close\r\n\r\n").toByteArray() + jpeg
                    )
                }
                output.flush()
            } catch (_: Exception) {
            } finally {
                try {
                    socket.close()
                } catch (_: Exception) {
                }
            }
            return
        }

        synchronized(clients) {
            clients.add(socket)
        }
        log.i("DownCam client connected: ${socket.inetAddress}")

        var lastSeq = -1L
        try {
            val output = socket.getOutputStream()
            output.write(
                ("HTTP/1.0 200 OK\r\n" +
                        "Content-Type: multipart/x-mixed-replace; boundary=downcamframe\r\n\r\n").toByteArray()
            )
            output.flush()

            while (running.get() && !socket.isClosed) {
                var jpeg: ByteArray? = null
                var seq: Long
                synchronized(frameLock) {
                    seq = frameSeq
                    jpeg = latestJpeg
                }

                val frameBytes = jpeg
                if (frameBytes == null || seq == lastSeq) {
                    Thread.sleep(100)
                    continue
                }

                lastSeq = seq
                val frame = ("--downcamframe\r\n" +
                        "Content-Type: image/jpeg\r\n" +
                        "Content-Length: ${frameBytes.size}\r\n\r\n").toByteArray() +
                        frameBytes + "\r\n\r\n".toByteArray()
                output.write(frame)
                output.flush()

                Thread.sleep(1000 / fpsLimit.toLong())
            }
        } catch (_: InterruptedException) {
        } catch (_: Exception) {
        } finally {
            synchronized(clients) {
                clients.remove(socket)
            }
            try {
                socket.close()
            } catch (_: Exception) {
            }
            log.i("DownCam client disconnected")
        }
    }

    // Read the request headers (up to the empty line) and return the request line,
    // e.g. "GET /snapshot HTTP/1.1"
    private fun readHttpRequestLine(socket: Socket): String {
        return try {
            val reader = socket.getInputStream().bufferedReader()
            reader.readLine() ?: ""
        } catch (_: Exception) {
            ""
        }
    }
}
