package nl.wur.mavdrone.utils

import android.content.SharedPreferences

class GenericPreferenceManager<T>(
    /**
     * Generic version of preference manager.
     * Source: https://alialp.medium.com/gain-flexibility-with-generic-shared-preferences-for-android-e3752cb042fd
     */
    private val sp: SharedPreferences
) {
    @Suppress("unchecked_cast")
    fun set(key: String?, value: T) {
        val editor = sp.edit()

        when (value) {
            is Boolean -> editor.putBoolean(key, value)
            is Set<*> -> editor.putStringSet(key, value as Set<String?>)
            is String -> editor.putString(key, value)
            is Float -> editor.putFloat(key, value)
            is Long -> editor.putLong(key, value)
            is Int -> editor.putInt(key, value)
            else -> throw IllegalArgumentException("Unsupported type")
        }

        editor.apply()
    }

    @Suppress("unchecked_cast")
    fun get(key: String?, defaultValue: T): T {
        return when (defaultValue) {
            is Boolean -> sp.getBoolean(key, defaultValue) as T
            is Set<*> -> sp.getStringSet(key, HashSet()) as T
            is String -> sp.getString(key, defaultValue) as T
            is Float -> sp.getFloat(key, defaultValue) as T
            is Long -> sp.getLong(key, defaultValue) as T
            is Int -> sp.getInt(key, defaultValue) as T
            else -> throw Exception("Cannot cast type!")
        }
    }
}
