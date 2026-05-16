package nl.wur.mavdrone.communication.utils

import android.content.Context
import android.os.StatFs
import com.github.mustachejava.DefaultMustacheFactory
import com.github.mustachejava.MustacheFactory
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil
import nl.wur.mavdrone.utils.Logger
import nl.wur.mavdrone.utils.RxException.WriteException
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.io.InputStreamReader

class FileUtils {
    companion object {
        @Throws(WriteException::class)
        fun writeFileUsingTemplate(
            templatePath: String,
            outputFile: String,
            content: Map<String, Any>,
            name: String,
            log: Logger
        ) {
            try {
                ContextUtil.getContext().assets.open(templatePath).use { templateStream ->
                    InputStreamReader(templateStream).use { reader ->
                        BufferedWriter(
                            FileWriter(outputFile, false), 8192
                        ).use { writer ->
                            log.i("Generating $name file: $outputFile")
                            val mf: MustacheFactory = DefaultMustacheFactory()
                            val mustache = mf.compile(reader, name)
                            mustache.execute(writer, content).flush()
                        }
                    }
                }
            } catch (e: IOException) {
                throw WriteException("Could not write " + name + " file " + outputFile + ": " + e.message).setLogger(
                    log
                )
            }
        }

        fun getAvailableDiskSpaceForFolder(folder: String): Long {
            val stat = StatFs(File(folder).absolutePath)
            val availableBlocks = stat.availableBlocksLong
            val blockSize = stat.blockSizeLong
            return availableBlocks * blockSize
        }

        fun getTotalDiskSpace(folder: String): Long {
            val stat = StatFs(File(folder).absolutePath)
            val totalBlocks = stat.blockCountLong
            val blockSize = stat.blockSizeLong
            return totalBlocks * blockSize
        }

        fun createFolderIfNeeded(path: String) {
            val folder = File(path)
            if (!folder.exists()) {
                folder.mkdir()
            }
        }

        fun getExternalCacheDirPath(subFolder: String): String {
            return getExternalCacheDirPath(ContextUtil.getContext(), subFolder)
        }

        fun getExternalCacheDirPath(context: Context, subFolder: String): String {
            return DiskUtil.getExternalCacheDirPath(context, subFolder)
        }
    }
}