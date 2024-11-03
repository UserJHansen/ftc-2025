package org.firstinspires.ftc.teamcode.messages

class StringMessage(@JvmField val message: String) {
    @JvmField
    val timestamp = System.nanoTime()
}