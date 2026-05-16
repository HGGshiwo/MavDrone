package nl.wur.mavdrone.utils

import kotlin.math.min

class ByteUtils {
    companion object {
        fun intToBytes(value: Int): ByteArray {
            val byteArray = ByteArray(4)
            byteArray[3] = ((value shr 24) and 0xFF).toByte()
            byteArray[2] = ((value shr 16) and 0xFF).toByte()
            byteArray[1] = ((value shr 8) and 0xFF).toByte()
            byteArray[0] = (value and 0xFF).toByte()
            return byteArray
        }

        fun intFromBytes(bytes: ByteArray): Int {
            val byteArray = ByteArray(4)
            System.arraycopy(
                bytes, 0, byteArray, 0, min(byteArray.size.toDouble(), bytes.size.toDouble())
                    .toInt()
            )

            return ((byteArray[3].toInt() and 0xFF) shl 24) or
                    ((byteArray[2].toInt() and 0xFF) shl 16) or
                    ((byteArray[1].toInt() and 0xFF) shl 8) or
                    ((byteArray[0].toInt() and 0xFF))
        }

        fun floatToBytes(value: Float): ByteArray {
            return intToBytes(value.toRawBits())
        }

        fun floatFromBytes(bytes: ByteArray): Float {
            return Float.fromBits(intFromBytes(bytes))
        }
    }
}
