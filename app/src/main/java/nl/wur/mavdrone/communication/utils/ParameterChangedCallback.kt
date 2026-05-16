package nl.wur.mavdrone.communication.utils

fun interface ParameterChangedCallback<T> {
    fun parameterChanged(newValue: T)
}
