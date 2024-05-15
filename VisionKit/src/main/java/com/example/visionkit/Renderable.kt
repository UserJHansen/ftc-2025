package com.example.visionkit

interface Renderable {
    val labels: List<Label>

    fun render(): List<Line<Point3D>>
}