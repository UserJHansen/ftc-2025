package org.firstinspires.ftc.teamcode.messages

import com.qualcomm.robotcore.hardware.NormalizedRGBA

class ColorMessage(color: NormalizedRGBA) {
    @JvmField
    val timestamp = System.nanoTime()

    @JvmField
    val r = color.red.toDouble()

    @JvmField
    val g = color.green.toDouble()

    @JvmField
    val b = color.blue.toDouble()

    @JvmField
    val a = color.alpha.toDouble()

    override fun toString(): String {
        return "$r $g $b $a"
    }
}