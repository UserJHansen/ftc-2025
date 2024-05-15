package com.example.visionkit

import java.awt.Color

class Line<T : Point<T>>(var p1: T, var p2: T, var colour: Color = Color.BLACK) {

    fun distance(p: T): Double {
        val v1 = p - this.p1
        val v2 = this.p2 - this.p1
        val v3 = v1.cross(v2)
        return v3.length / v2.length
    }

    override fun toString(): String {
        return "Line($p1, $p2)"
    }
}
