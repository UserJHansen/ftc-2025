package org.firstinspires.ftc.teamcode.messages

class BooleanMessage(@JvmField val message: Boolean) {
    @JvmField
    val timestamp = System.nanoTime()
}