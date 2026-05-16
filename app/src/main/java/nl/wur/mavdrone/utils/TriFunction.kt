package nl.wur.mavdrone.utils

import java.util.function.Function

fun interface TriFunction<A1, A2, A3, R> {
    fun apply(t: A1, u: A2, v: A3): R

    fun <K> andThen(after: Function<in R, out K>): TriFunction<A1, A2, A3, K> {
        return TriFunction { t: A1, u: A2, v: A3 -> after.apply(apply(t, u, v)) }
    }
}
