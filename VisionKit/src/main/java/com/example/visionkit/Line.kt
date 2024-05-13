package com.example.visionkit

class Line {
    var p1: Point3D = Point3D(0.0, 0.0, 0.0)
    var p2: Point3D = Point3D(0.0, 0.0, 1.0)

    constructor(p1: Point3D, p2: Point3D) {
        this.p1 = p1
        this.p2 = p2
    }

    fun distance(p: Point3D): Double {
        val v1 = p - this.p1
        val v2 = this.p2 - this.p1
        val v3 = v1.cross(v2)
        return v3.length / v2.length
    }

    override fun toString(): String {
        return "Line($p1, $p2)"
    }
}