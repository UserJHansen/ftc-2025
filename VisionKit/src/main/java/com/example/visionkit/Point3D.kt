package com.example.visionkit

import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

open class Point3D(var x: Double, var y: Double, var z: Double) : Point<Point3D> {

    override var length: Double
        get() = sqrt(this.x * this.x + this.y * this.y + this.z * this.z)
        set(value) {
            val currentLength = this.length
            if (currentLength != 0.0) {
                this.x = this.x / currentLength * value
                this.y = this.y / currentLength * value
                this.z = this.z / currentLength * value
            }
        }

    var unit: Point3D
        get() = this / this.length
        set(value) {
            this.length = value.length
        }

    var theta: Double
        get() {
            val theta = atan2(this.y, this.x)
            return if (theta < 0) {
                theta + 2 * Math.PI
            } else if (theta >= 2 * Math.PI) {
                theta - 2 * Math.PI
            } else {
                theta
            }
        }
        set(value) {
            val currentLength = this.length
            val currentPhi = this.phi
            this.x = cos(value) * sin(currentPhi) * currentLength
            this.y = sin(value) * sin(currentPhi) * currentLength
        }

    var phi: Double
        get() {
            val phi = acos(this.z / this.length)
            return if (phi < 0) {
                phi + 2 * Math.PI
            } else if (phi > 2*Math.PI) {
                phi - 2 * Math.PI
            } else {
                phi
            }
        }
        set(value) {
            val currentLength = this.length
            val currentTheta = this.theta
            this.x = cos(currentTheta) * sin(value) * currentLength
            this.y = sin(currentTheta) * sin(value) * currentLength
            this.z = cos(value) * currentLength
        }

    override operator fun plus(p: Point3D): Point3D {
        return Point3D(this.x + p.x, this.y + p.y, this.z + p.z)
    }

    override operator fun minus(p: Point3D): Point3D {
        return Point3D(this.x - p.x, this.y - p.y, this.z - p.z)
    }

    operator fun times(scalar: Double): Point3D {
        return Point3D(this.x * scalar, this.y * scalar, this.z * scalar)
    }

    operator fun div(scalar: Double): Point3D {
        return Point3D(this.x / scalar, this.y / scalar, this.z / scalar)
    }

    operator fun timesAssign(scalar: Double) {
        this.x *= scalar
        this.y *= scalar
        this.z *= scalar
    }

    operator fun divAssign(scalar: Double) {
        this.x /= scalar
        this.y /= scalar
        this.z /= scalar
    }

    operator fun unaryMinus(): Point3D {
        return Point3D(-this.x, -this.y, -this.z)
    }

    override fun distance(p: Point3D): Double {
        return (this - p).length
    }

    fun copy(): Point3D {
        return Point3D(this.x, this.y, this.z)
    }

    fun normalize() {
        this.length = 1.0
    }

    override fun dot(p: Point3D): Double {
        return this.x * p.x + this.y * p.y + this.z * p.z
    }

    override fun cross(p: Point3D): Point3D {
        return Point3D(
            this.y * p.z - this.z * p.y,
            this.z * p.x - this.x * p.z,
            this.x * p.y - this.y * p.x
        )
    }

    fun projectOnto(p: Point3D): Point3D {
        return p * (this.dot(p) / p.length)
    }

         open fun rotateAround(axis: Point3D, angle: Double): Point3D {
//        WARNING: This was written by AI and has yet to be completely verified
//        I have given it a quick look and it seems to match https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
//        but I have not tested it

        val cosAngle = cos(angle)
        val sinAngle = sin(angle)
        val oneMinusCosAngle = 1 - cosAngle

        val rotatedX = (cosAngle + oneMinusCosAngle * axis.x * axis.x) * this.x +
                (oneMinusCosAngle * axis.x * axis.y - axis.z * sinAngle) * this.y +
                (oneMinusCosAngle * axis.x * axis.z + axis.y * sinAngle) * this.z

        val rotatedY = (oneMinusCosAngle * axis.x * axis.y + axis.z * sinAngle) * this.x +
                (cosAngle + oneMinusCosAngle * axis.y * axis.y) * this.y +
                (oneMinusCosAngle * axis.y * axis.z - axis.x * sinAngle) * this.z

        val rotatedZ = (oneMinusCosAngle * axis.x * axis.z - axis.y * sinAngle) * this.x +
                (oneMinusCosAngle * axis.y * axis.z + axis.x * sinAngle) * this.y +
                (cosAngle + oneMinusCosAngle * axis.z * axis.z) * this.z

        return Point3D(rotatedX, rotatedY, rotatedZ)
    }

    fun angleBetween(p: Point3D): Double {
        return acos(this.dot(p) / (this.length * p.length))
    }

    override fun toString(): String {
        return "Point3D(%.3f, %.3f, %.3f)".format(x, y, z)
    }
}