package com.example.visionkit

interface Point<T> {
    val length: Double

    operator fun minus(p: T): T
    operator fun plus(p: T): T


    fun cross(p: T): Point3D
    fun dot(p: T): Double
    fun distance(p2: T): Double
}