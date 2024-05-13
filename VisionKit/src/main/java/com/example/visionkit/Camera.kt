package com.example.visionkit

class Camera(var position: Point3D, var direction: Rotation3D, var up: Rotation3D, var fov: Double) {
//    Cameras are line renderers

    fun render(point: Point3D): List<Line> {
//        Make a cube around the point
        val x = point.x
        val y = point.y
        val z = point.z

        var lines = mutableListOf<Line>()
        val points = arrayOf(
            Point3D(x - 1, y - 1, z - 1),
            Point3D(x + 1, y - 1, z - 1),
            Point3D(x + 1, y + 1, z - 1),
            Point3D(x - 1, y + 1, z - 1),
            Point3D(x - 1, y - 1, z + 1),
            Point3D(x + 1, y - 1, z + 1),
            Point3D(x + 1, y + 1, z + 1),
            Point3D(x - 1, y + 1, z + 1)
        )

        for (i in 0 until 4) {
            lines.add(Line(points[i], points[(i + 1) % 4]))
            lines.add(Line(points[i + 4], points[(i + 1) % 4 + 4]))
            lines.add(Line(points[i], points[i + 4]))
        }

        return lines
    }

    fun render(points: List<Point3D>): List<Line> {
        var lines = mutableListOf<Line>()
        for (i in 0 until points.size) {
            lines.addAll(render(points[i]))
        }
        return lines
    }

    override fun toString(): String {
        return "Camera(position=$position, direction=$direction, fov=%.2f)".format(fov)
    }
}