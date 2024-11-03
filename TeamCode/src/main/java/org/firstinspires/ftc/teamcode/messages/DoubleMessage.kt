package org.firstinspires.ftc.teamcode.messages

class DoubleMessage(@JvmField val message: Double) {
    @JvmField
    val timestamp = System.nanoTime()
}