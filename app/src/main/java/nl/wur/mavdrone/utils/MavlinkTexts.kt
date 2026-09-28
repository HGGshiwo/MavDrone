package nl.wur.mavdrone.utils

object MavlinkTexts {
    private const val STATUSTEXT_MAX_BYTES = 50

    /**
     * MAVLink STATUSTEXT carries at most 50 bytes. Truncating by character count lets
     * the MAVLink serializer cut a multi-byte UTF-8 sequence in half, and GCS
     * implementations that put the text into JSON then abort (nlohmann
     * type_error.316). Truncate on a byte boundary without splitting UTF-8 sequences.
     */
    fun truncateStatustext(text: String): String {
        val bytes = text.toByteArray(Charsets.UTF_8)
        if (bytes.size <= STATUSTEXT_MAX_BYTES) {
            return text
        }

        var end = STATUSTEXT_MAX_BYTES
        while (end > 0 && (bytes[end].toInt() and 0xC0) == 0x80) {
            end--
        }
        return bytes.copyOfRange(0, end).toString(Charsets.UTF_8)
    }
}
