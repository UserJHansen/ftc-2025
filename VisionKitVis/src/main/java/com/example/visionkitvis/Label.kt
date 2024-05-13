package com.example.visionkitvis

import com.example.visionkit.Point3D

class Label {
    var text: String = ""
    var position: Point3D = Point3D(0.0, 0.0, 0.0)

    constructor(text: String, position: Point3D) {
        this.text = text
        this.position = position
    }
}