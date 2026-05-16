package nl.wur.mavdrone.communication.utils

class FTPProtocol {
    companion object {
        const val FTP_SEQ_START_BYTE: Int = 0
        const val FTP_SEQ_END_BYTE: Int = 1
        const val FTP_SESSION_ID_BYTE: Int = 2
        const val FTP_OPCODE_BYTE: Int = 3
        const val FTP_SIZE_BYTE: Int = 4
        const val FTP_REQ_OPCODE_BYTE: Int = 5
        const val FTP_BURST_COMPLETE_BYTE: Int = 6
        const val FTP_PADDING_BYTE: Int = 7
        const val FTP_OFFSET_START_BYTE: Int = 8
        const val FTP_OFFSET_END_BYTE: Int = 11
        const val FTP_DATA_START_BYTE: Int = 12
        const val FTP_DATA_END_BYTE: Int = 251

        const val NONE: Int = 0
        const val TERMINATE_SESSION: Int = 1
        const val RESET_SESSIONS: Int = 2
        const val LIST_DIRECTORY: Int = 3
        const val OPEN_FILE_RO: Int = 4
        const val READ_FILE: Int = 5
        const val CREATE_FILE: Int = 6
        const val WRITE_FILE: Int = 7
        const val REMOVE_FILE: Int = 8
        const val CREATE_DIRECTORY: Int = 9
        const val REMOVE_DIRECTORY: Int = 10
        const val OPEN_FILE_WO: Int = 11
        const val TRUNCATE_FILE: Int = 12
        const val RENAME: Int = 13
        const val CALC_FILE_CRC32: Int = 14
        const val BURST_READ_FILE: Int = 15
        const val ACK: Int = 128
        const val NAK: Int = 129

        const val FAIL: Int = 1
        const val FAIL_ERRNO: Int = 2
        const val INVALID_DATA_SIZE: Int = 3
        const val INVALID_SESSION: Int = 4
        const val NO_SESSION_AVAILABLE: Int = 5
        const val EOF: Int = 6
        const val UNKNOWN_COMMAND: Int = 7
        const val FILE_EXISTS: Int = 8
        const val FILE_PROTECTED: Int = 9
        const val FILE_NOT_FOUND: Int = 10
    }
}
