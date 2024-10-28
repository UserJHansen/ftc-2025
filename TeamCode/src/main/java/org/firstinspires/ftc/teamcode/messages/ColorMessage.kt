package org.firstinspires.ftc.teamcode.messages

import com.qualcomm.robotcore.hardware.NormalizedRGBA

class ColorMessage(color: NormalizedRGBA) {
    @JvmField val timestamp = System.nanoTime();
    @JvmField val r = color.red;
    @JvmField val g = color.green;
    @JvmField val b = color.blue;
    @JvmField val a = color.alpha;

    override fun toString(): String {
        return "$r $g $b $a"
    }
}