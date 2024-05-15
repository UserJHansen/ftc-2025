package com.example.visionkit.objects

import com.example.visionkit.Label
import com.example.visionkit.Line
import com.example.visionkit.Point3D
import com.example.visionkit.Renderable

class FieldPerimeter : Renderable {
    override val labels = mutableListOf<Label>()
        .also { arr ->
            arr.addAll((0..10).map {
                Label("%.2f".format((it - 5) * 0.36), Point3D((it - 5) * 0.36, 0.0, 0.0))
            })
        }.also { arr ->
            arr.addAll((0..10).map {
                Label("%.2f".format((it - 5) * 0.36), Point3D(0.0, (it - 5) * 0.36, 0.0))
            })
        }

    override fun render(): List<Line<Point3D>> {
//        Divide the 3.6m field into 6x6 grid
        val lines = mutableListOf<Line<Point3D>>()
        val size = 3.6 / 6

        for (i in 0..6) {
            lines.add(Line(Point3D(-1.8 + i * size, -1.8, 0.0), Point3D(-1.8 + i * size, 1.8, 0.0)))
            lines.add(Line(Point3D(-1.8, -1.8 + i * size, 0.0), Point3D(1.8, -1.8 + i * size, 0.0)))
        }

//        Add the 1ft field walls
        lines.add(Line(Point3D(-1.8, -1.8, 0.3), Point3D(1.8, -1.8, 0.3)))
        lines.add(Line(Point3D(-1.8, 1.8, 0.3), Point3D(1.8, 1.8, 0.3)))
        lines.add(Line(Point3D(-1.8, -1.8, 0.3), Point3D(-1.8, 1.8, 0.3)))
        lines.add(Line(Point3D(1.8, -1.8, 0.3), Point3D(1.8, 1.8, 0.3)))

//        Corners
        lines.add(Line(Point3D(-1.8, -1.8, 0.3), Point3D(-1.8, -1.8, 0.0)))
        lines.add(Line(Point3D(1.8, -1.8, 0.3), Point3D(1.8, -1.8, 0.0)))
        lines.add(Line(Point3D(-1.8, 1.8, 0.3), Point3D(-1.8, 1.8, 0.0)))
        lines.add(Line(Point3D(1.8, 1.8, 0.3), Point3D(1.8, 1.8, 0.0)))

        return lines
    }

}