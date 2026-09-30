package nl.wur.mavdrone.communication.utils

/**
 * Fuses the GPS-derived local position (noisy, bounded error) with the DJI fused
 * velocity (smooth, drifting when integrated) into a smooth local position, per
 * axis (north / east). This is a 2-state linear Kalman filter with online
 * covariance propagation:
 *
 *   predict:  pos += vel * dt          (velocity state propagated with covariance)
 *   velocity: vel += K2 * (v_meas - vel)   every frame, small R
 *   position: pos += K1 * (z - pos)        every GPS-derived sample, gated at 9 sigma
 *
 * Model caveat: motion is followed through the velocity input (updated each frame),
 * not assumed constant between updates; acceleration error is ~0.5*a*dt^2.
 * When no velocity is available the filter degrades to GPS pass-through instead of
 * producing fake smoothness.
 */
class LocalPositionFilter {
    private val north = AxisFilter()
    private val east = AxisFilter()
    private var lastUpdateMillis = 0L

    /**
     * Feed one sample. [north], [east] is the GPS-derived position, [velNorth],
     * [velEast] the DJI fused velocity ([hasVelocity] false when the key is not
     * pushed). Returns the filtered position, or the raw position when the filter
     * is pass-through degraded.
     */
    fun update(
        north: Double,
        east: Double,
        velNorth: Double,
        velEast: Double,
        hasVelocity: Boolean
    ): Pair<Double, Double> {
        val now = System.currentTimeMillis()
        val dt = if (lastUpdateMillis == 0L) 0.0 else (now - lastUpdateMillis) / 1000.0
        lastUpdateMillis = now

        // Long gap or first sample: re-lock directly onto the observation
        if (!this.north.initialized || !this.east.initialized || dt <= 0.0 || dt > MAX_DT_SECONDS) {
            reset(north, east)
            return Pair(north, east)
        }

        this.north.predict(dt)
        this.east.predict(dt)

        if (hasVelocity) {
            this.north.updateVelocity(velNorth)
            this.east.updateVelocity(velEast)
        } else {
            this.north.degrade()
            this.east.degrade()
        }

        this.north.updatePosition(north)
        this.east.updatePosition(east)

        return Pair(this.north.pos, this.east.pos)
    }

    fun reset(north: Double, east: Double) {
        this.north.reset(north)
        this.east.reset(east)
        lastUpdateMillis = System.currentTimeMillis()
    }

    private class AxisFilter {
        var initialized = false
        var pos = 0.0
        var vel = 0.0
        var p11 = 1.0
        var p12 = 0.0
        var p21 = 0.0
        var p22 = 1.0
        var rejections = 0

        fun reset(observation: Double) {
            pos = observation
            vel = 0.0
            // Start with a large covariance so the filter locks onto observations fast
            p11 = 4.0
            p12 = 0.0
            p21 = 0.0
            p22 = 1.0
            rejections = 0
            initialized = true
        }

        fun predict(dt: Double) {
            pos += vel * dt
            p11 += (p12 + p21) * dt + p22 * dt * dt + ACCEL_NOISE_VAR * dt
            p12 += p22 * dt
            p21 += p22 * dt
            p22 += VELOCITY_RANDOM_WALK_VAR * dt
        }

        fun updateVelocity(measured: Double) {
            val s = p22 + VELOCITY_NOISE_VAR
            if (s <= 0.0) return
            val k1 = p12 / s
            val k2 = p22 / s
            val y = measured - vel
            pos += k1 * y
            vel += k2 * y
            val n11 = p11 - k1 * p21
            val n12 = p12 - k1 * p22
            val n21 = p21 - k2 * p21
            val n22 = p22 - k2 * p22
            p11 = n11; p12 = n12; p21 = n21; p22 = n22
        }

        fun updatePosition(observed: Double) {
            val s = p11 + POSITION_NOISE_VAR
            if (s <= 0.0) return
            val y = observed - pos

            // Outlier gate: reject observations further than 9 sigma, re-lock after
            // too many consecutive rejections (e.g. the home point was reset)
            if (y * y > OUTLIER_SIGMA * OUTLIER_SIGMA * s) {
                rejections++
                if (rejections >= FORCE_RELOCK_REJECTIONS) {
                    reset(observed)
                }
                return
            }
            rejections = 0

            val k1 = p11 / s
            val k2 = p21 / s
            pos += k1 * y
            vel += k2 * y
            val n11 = (1.0 - k1) * p11
            val n12 = (1.0 - k1) * p12
            val n21 = p21 - k2 * p11
            val n22 = p22 - k2 * p12
            p11 = n11; p12 = n12; p21 = n21; p22 = n22
        }

        // Velocity input missing: inflate the velocity covariance so subsequent
        // position updates trust the observation more (pass-through degradation)
        fun degrade() {
            p22 *= DEGRADE_FACTOR
        }

        companion object {
            private const val POSITION_NOISE_VAR = 1.0
            private const val VELOCITY_NOISE_VAR = 0.04
            private const val ACCEL_NOISE_VAR = 0.5
            private const val VELOCITY_RANDOM_WALK_VAR = 1e-3
            private const val OUTLIER_SIGMA = 9.0 / 3.0 // 9 sigma in 3-sigma units
            private const val FORCE_RELOCK_REJECTIONS = 10
            private const val DEGRADE_FACTOR = 1.5
        }
    }

    companion object {
        private const val MAX_DT_SECONDS = 0.5
    }
}
