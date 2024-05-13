package com.example.visionkit

class Rotation3D: Point3D {
    constructor(x: Double, y: Double, z: Double): super(x, y, z)

    constructor(p: Point3D): super(p.x, p.y, p.z)

    override fun rotateAround(axis: Point3D, angle: Double): Rotation3D {
        return Rotation3D(super.rotateAround(axis, angle))
    }

    override fun toString(): String {
        return "Rotation3D(%.2f, %.2f, %.2f)".format(x, y, z)
    }
}