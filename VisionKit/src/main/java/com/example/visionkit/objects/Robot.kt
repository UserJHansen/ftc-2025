package com.example.visionkit.objects

import com.example.visionkit.Camera
import com.example.visionkit.Label
import com.example.visionkit.Line
import com.example.visionkit.Point3D
import com.example.visionkit.Renderable

class Robot(val cameras: List<Camera>) : Renderable {
    override val labels = listOf(Label("Robot", Point3D(0.0, 0.0, 0.0)))
    private val size: Double = 0.35

    //    Automatically update label position
    var position: Point3D = Point3D(0.0, 0.0, 0.0)
        set(value) {
            field = value
            labels[0].position = value
        }

    override fun render(): List<Line<Point3D>> {
//        Make a cube around the point
        val x = position.x
        val y = position.y

        val lines = mutableListOf<Line<Point3D>>()
        val points = arrayOf(
            Point3D(x - (size / 2), y - (size / 2), 0.0),
            Point3D(x + (size / 2), y - (size / 2), 0.0),
            Point3D(x + (size / 2), y + (size / 2), 0.0),
            Point3D(x - (size / 2), y + (size / 2), 0.0),
            Point3D(x - (size / 2), y - (size / 2), size),
            Point3D(x + (size / 2), y - (size / 2), size),
            Point3D(x + (size / 2), y + (size / 2), size),
            Point3D(x - (size / 2), y + (size / 2), size)
        )

        for (i in 0 until 4) {
            lines.add(Line(points[i], points[(i + 1) % 4]))
            lines.add(Line(points[i + 4], points[(i + 1) % 4 + 4]))
            lines.add(Line(points[i], points[i + 4]))
        }

        return lines
    }
}