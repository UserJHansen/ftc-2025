package com.example.visionkit

import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Point2D(var x: Double, var y: Double) : Point<Point2D> {

    override var length: Double
        get() = sqrt(this.x * this.x + this.y * this.y)
        set(value) {
            val currentLength = this.length
            if (currentLength != 0.0) {
                this.x = this.x / currentLength * value
                this.y = this.y / currentLength * value
            }
        }

    var angle: Double
        get() = atan2(this.y, this.x)
        set(value) {
            val currentLength = this.length
            this.x = cos(value) * currentLength
            this.y = sin(value) * currentLength
        }

    override operator fun plus(p: Point2D): Point2D {
        return Point2D(this.x + p.x, this.y + p.y)
    }

    override operator fun minus(p: Point2D): Point2D {
        return Point2D(this.x - p.x, this.y - p.y)
    }

    operator fun times(scalar: Double): Point2D {
        return Point2D(this.x * scalar, this.y * scalar)
    }

    operator fun div(scalar: Double): Point2D {
        return Point2D(this.x / scalar, this.y / scalar)
    }

    operator fun plusAssign(p: Point2D) {
        this.x += p.x
        this.y += p.y
    }

    operator fun minusAssign(p: Point2D) {
        this.x -= p.x
        this.y -= p.y
    }

    operator fun timesAssign(scalar: Double) {
        this.x *= scalar
        this.y *= scalar
    }

    operator fun divAssign(scalar: Double) {
        this.x /= scalar
        this.y /= scalar
    }

    operator fun unaryMinus(): Point2D {
        return Point2D(-this.x, -this.y)
    }

    override fun distance(p: Point2D): Double {
        return (this - p).length
    }

    fun copy(): Point2D {
        return Point2D(this.x, this.y)
    }

    fun normalize() {
        this.length = 1.0
    }

    override fun dot(p: Point2D): Double {
        return this.x * p.x + this.y * p.y
    }

    override fun cross(p: Point2D): Point3D {
        return Point3D(0.0, 0.0, this.x * p.y - this.y * p.x)
    }

    fun rotate(angle: Double): Point2D {
        val cos = cos(angle)
        val sin = sin(angle)
        return Point2D(this.x * cos - this.y * sin, this.x * sin + this.y * cos)
    }

    fun rotateAround(pivot: Point2D, angle: Double): Point2D {
        return (this - pivot).rotate(angle) + pivot
    }

    fun angle(p: Point2D): Double {
        return acos(this.dot(p) / (this.length * p.length))
    }

    fun projectOnto(p: Point2D): Point2D {
        return p * (this.dot(p) / p.length)
    }

    fun reflect(normal: Point2D): Point2D {
        return this - normal * 2.0 * this.dot(normal)
    }

    override fun toString(): String {
        return "Point2D(%.3f, %.3f)".format(x, y)
    }
}