package com.example.visionkitvis

import com.example.visionkit.Camera
import com.example.visionkit.Line

interface Renderable {
    val label: Label

    fun render(camera: Camera): List<Line>
}