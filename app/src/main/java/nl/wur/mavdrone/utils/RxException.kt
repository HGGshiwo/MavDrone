package nl.wur.mavdrone.utils

import dji.v5.common.error.IDJIError

open class RxException(msg: String?) : Exception(msg) {
    var logTag: String = ""
        private set
    var showToast: Boolean = true
        private set

    fun setLogTag(logTag: String?): RxException {
        this.logTag = logTag ?: ""
        return this
    }

    fun setLogger(logger: Logger): RxException {
        this.logTag = logger.tag()
        return this
    }

    fun setShowToast(showToast: Boolean): RxException {
        this.showToast = showToast
        return this
    }

    class ReferenceNotFoundException(msg: String) : RxException(msg)
    class MissionConverterException(msg: String) : RxException(msg)
    class FenceMissionConverterException(msg: String) : RxException(msg)
    class WriteException(msg: String) : RxException(msg)
    class NoMissionUploadedException(msg: String) : RxException(msg)
    class DroneNotArmedException(msg: String) : RxException(msg)
    class DroneNotExecutingMissionException(msg: String) : RxException(msg)
    class MissionUploadException(msg: String) : RxException(msg)
    class NotEnoughWaypointsException(msg: String) : RxException(msg)

    class DJIException(msg: String) : RxException(msg) {
        companion object {
            fun fromDJI(error: IDJIError): DJIException {
                return DJIException(
                    "DJI error (${error.errorType()}): errorcode=${error.errorCode()} hint=${error.hint()} description=${error.description()} innercode=${error.innerCode()}"
                )
            }
        }
    }
}
