package com.example.visionkitvis

import com.example.visionkit.Camera
import com.example.visionkit.Line
import com.example.visionkit.Point3D

class Box : Renderable {
    var position: Point3D = Point3D(0.0, 0.0, 0.0)
    override val label = Label("Box", position)

    override fun render(camera: Camera): List<Line> {
        return camera.render(this.position)
    }
}