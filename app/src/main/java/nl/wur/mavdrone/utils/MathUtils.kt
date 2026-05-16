package nl.wur.mavdrone.utils

fun Double.toRadians(): Double {
    return Math.toRadians(this)
}

fun Double.toDegrees(): Double {
    return Math.toDegrees(this)
}

fun Float.toRadians(): Float {
    return Math.toRadians(this.toDouble()).toFloat()
}

fun Float.toDegrees(): Float {
    return Math.toDegrees(this.toDouble()).toFloat()
}
