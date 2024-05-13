package com.example.visionkit

import kotlin.math.abs

class Plane {
    var origin: Point3D = Point3D(0.0, 0.0, 0.0)
    var normal: Point3D = Point3D(0.0, 0.0, 1.0)
    var sideBase: Point3D = Point3D(1.0, 0.0, 0.0) // Must be orthogonal to normal

    constructor(origin: Point3D, normal: Rotation3D, sideBase: Point3D) {
        this.origin = origin
        this.normal = normal
        if (normal.dot(sideBase) != 0.0) {
            throw IllegalArgumentException("side_base must be orthogonal to normal")
        }
        this.sideBase = sideBase
    }

    fun fromPoints(p1: Point3D, p2: Point3D, p3: Point3D) {
        val v1 = p2 - p1
        val v2 = p3 - p1
        this.origin = p1
        this.normal = v1.cross(v2)
        this.normal.length = 1.0

        val v3 = this.normal.cross(v1)
        this.sideBase = v3
    }

    fun distance(p: Point3D): Double {
        return abs((p - this.origin).dot(this.normal))
    }

    fun project(p: Point3D): Point3D {
        val v = p - this.origin
        return p - this.normal * v.dot(this.normal)
    }

    fun map(p: Point2D): Point3D {
        return this.origin + this.sideBase * p.x + this.normal * p.y
    }

    override fun toString(): String {
        return "Plane($origin, $normal)"
    }
}