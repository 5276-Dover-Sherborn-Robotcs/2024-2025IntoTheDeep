package org.firstinspires.ftc.teamcode.util

import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.sqrt

data class Pose2D(@JvmField val x: Double, @JvmField val y: Double, @JvmField val h: Double) {
    operator fun plus(p: Pose2D): Pose2D {
        return Pose2D(x + p.x, y + p.y, (h + p.h) % (2* PI))
    }
    operator fun minus(p: Pose2D) = Pose2D(x - p.x, y - p.y, (h - p.h) % (2 * PI))

    operator fun times(d: Double) = Pose2D(x * d, y * d, (h * d) % (2 * PI))
    operator fun div(d: Double) = Pose2D(x / d, y / d, (h / d) % (2 * PI))

    // Custom implementation of ==,
    override fun equals(other: Any?): Boolean {
        return if (other is Pose2D) {
            other.x == x && other.y == y && other.h == h
        } else {
            false
        }
    }

    // This is just a simple distance formula
    fun dist(p: Pose2D) = sqrt(((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y)))

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + h.hashCode()
        return result
    }

}