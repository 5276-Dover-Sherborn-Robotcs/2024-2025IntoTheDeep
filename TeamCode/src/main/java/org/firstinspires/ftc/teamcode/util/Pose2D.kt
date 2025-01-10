package org.firstinspires.ftc.teamcode.util

import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.reflect.typeOf

class Pose2D(@JvmField val x: Double, @JvmField val y: Double, @JvmField val h: Double) {
    operator fun plus(p: Pose2D) = Pose2D(x + p.x, y + p.y, h + p.h)
    operator fun minus(p: Pose2D) = Pose2D(x - p.x, y - p.y, h - p.h)

    operator fun times(d: Double) = Pose2D(x * d, y * d, h * d)
    operator fun div(d: Double) = Pose2D(x / d, y / d, h / d)

    override fun equals(other: Any?): Boolean {
        return if (other is Pose2D) {
            other.x == x && other.y == y && other.h == h
        } else {
            false
        }
    }

    fun dist(p: Pose2D) = hypot((p.x - x), (p.y - y))

}