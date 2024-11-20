package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction

typealias Color = RevBlinkinLedDriver.BlinkinPattern

class StaticLights {
    companion object {
        @JvmStatic
        var colors: Array<Color> = arrayOf()

        @JvmStatic
        var blinksPerSecond = 4

        private var driver: RevBlinkinLedDriver? = null;

        @JvmStatic
        fun setup(hardwareMap: HardwareMap, name: String) {
            driver = hardwareMap.get(RevBlinkinLedDriver::class.java, name)
        }

        private var inSpecialState = false
        private var specialStateColors: Array<Color> = arrayOf()
        private var specialStateBlinksPerSecond = 8

        @JvmStatic
        fun specialState(colors: Array<Color>, blinksPerSecond: Int) {
            specialStateColors = colors
            specialStateBlinksPerSecond = blinksPerSecond
            inSpecialState = true
        }

        private var lastColor: Color? = null

        @JvmStatic
        fun update() {
            val currentColors = if (inSpecialState) specialStateColors else colors
            val currentBlinksPerSecond = if (inSpecialState) specialStateBlinksPerSecond else blinksPerSecond

            val newColor =
                currentColors[(
                        System.currentTimeMillis() * currentBlinksPerSecond
                                / 1_000_000).toInt() % currentColors.size]
            if (lastColor != newColor) {
                driver?.setPattern(newColor)
            }
            inSpecialState = false
        }

        @JvmStatic
        fun setIndex(index: Int, color: Color): LoggableAction {
            return Loggable("SET_${index}_TO_${color.name}", InstantAction {
                colors[index] = color
            })
        }

        @JvmStatic
        fun setColours(newColors: Array<Color>): LoggableAction {
            return Loggable("SET_COLORS_TO_ARRAY", InstantAction {
                colors = newColors
            })
        }
    }
}