package com.example.visionkit.objects

import com.example.visionkit.Label
import com.example.visionkit.Line
import com.example.visionkit.Plane
import com.example.visionkit.Point
import com.example.visionkit.Point3D
import com.example.visionkit.Renderable
import java.awt.Color

class PlaneRenderer<T : Point<T>>(
    private val plane: Plane,
    public var lines: List<Line<T>>,
    val label_text: String = "Plane Origin"
) :
    Renderable {
    override val labels = listOf(Label(this.label_text, plane.origin))

    override fun render(): List<Line<Point3D>> {
//        Draw Axis
        val size = 0.1
        val lines = mutableListOf<Line<Point3D>>()

        lines.add(
            Line(
                plane.map(Point3D(-size, 0.0, 0.0)),
                plane.map(Point3D(size, 0.0, 0.0)),
                Color.RED
            )
        )
        lines.add(
            Line(
                plane.map(Point3D(0.0, -size, 0.0)),
                plane.map(Point3D(0.0, size, 0.0)),
                Color.GREEN
            )
        )
        lines.add(
            Line(
                plane.map(Point3D(0.0, 0.0, -size)),
                plane.map(Point3D(0.0, 0.0, size)),
                Color.BLUE
            )
        )

        for (line in this.lines) {
            lines.add(
                Line(
                    plane.map(line.p1),
                    plane.map(line.p2)
                )
            )
        }

        return lines
    }
}