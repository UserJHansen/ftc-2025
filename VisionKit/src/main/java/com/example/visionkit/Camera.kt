package com.example.visionkit

import kotlin.math.atan

class Camera(
    var position: Point3D,
    var direction: Rotation3D,
    var up: Rotation3D,
    val fov: Double,
    val fx: Double,
    val fy: Double
) {
    fun convertToScreenCoords(point: Point3D): Point2D {
        var p = point - this.position
        p = Point3D(
            p.dot(this.direction),
            p.dot(this.up.cross(this.direction)),
            p.dot(this.up)
        )

        return Point2D(
            atan(p.y / p.x) * Math.PI * fx,
            -atan(p.z / p.x) * Math.PI * fy
        )
    }

    fun render(renderables: List<Renderable>): List<Line<Point2D>> {
        val lines = renderables.flatMap { it.render() }.toList()

        return lines.mapNotNull { line ->
            val p1 = line.p1 - this.position
            val p2 = line.p2 - this.position

//            Convert the 3d points onto the camera, using the camera's position, direction and fov

            if (p1.dot(this.direction) < 0.0 && p2.dot(this.direction) < 0.0) {
                return@mapNotNull null
            } else if (p1.angleBetween(p2) > Math.toRadians(this.fov)) {
                return@mapNotNull null
            } else if (p1.angleBetween(this.direction) > Math.toRadians(this.fov)) {
                return@mapNotNull null
            } else if (p2.angleBetween(this.direction) > Math.toRadians(this.fov)) {
                return@mapNotNull null
            }

            Line(
                convertToScreenCoords(line.p1),
                convertToScreenCoords(line.p2),
                line.colour
            )
        }
    }

    override fun toString(): String {
        return "Camera(position=$position, direction=$direction, fov=%.2f)".format(fov)
    }
}