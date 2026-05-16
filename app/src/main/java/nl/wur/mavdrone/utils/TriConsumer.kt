package nl.wur.mavdrone.utils


fun interface TriConsumer<T, U, V> {
    fun accept(t: T, u: U, v: V)

    fun andThen(after: TriConsumer<in T, in U, in V>): TriConsumer<T, U, V> {
        return TriConsumer { a: T, b: U, c: V ->
            accept(a, b, c)
            after.accept(a, b, c)
        }
    }
}