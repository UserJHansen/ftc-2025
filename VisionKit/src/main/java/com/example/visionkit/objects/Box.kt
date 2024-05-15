package com.example.visionkit.objects

import com.example.visionkit.Label
import com.example.visionkit.Line
import com.example.visionkit.Point3D
import com.example.visionkit.Renderable

class Box(var position: Point3D, var size: Double = 0.1, text: String = "Box") : Renderable {
    override val labels = listOf(Label(text, position))

    override fun render(): List<Line<Point3D>> {
//        Make a cube around the point
        val x = position.x
        val y = position.y
        val z = position.z

        val lines = mutableListOf<Line<Point3D>>()
        val points = arrayOf(
            Point3D(x - size, y - size, z - size),
            Point3D(x + size, y - size, z - size),
            Point3D(x + size, y + size, z - size),
            Point3D(x - size, y + size, z - size),
            Point3D(x - size, y - size, z + size),
            Point3D(x + size, y - size, z + size),
            Point3D(x + size, y + size, z + size),
            Point3D(x - size, y + size, z + size)
        )

        for (i in 0 until 4) {
            lines.add(Line(points[i], points[(i + 1) % 4]))
            lines.add(Line(points[i + 4], points[(i + 1) % 4 + 4]))
            lines.add(Line(points[i], points[i + 4]))
        }

        return lines
    }
}