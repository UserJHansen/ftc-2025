package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs


// The full rotation time is the amount of time for the servo to do a full rotation, used in the
// actions that change the position to give it a realistic delay.
//
// Some good values:
// - GoBilda High Torque servo: 1.17
// - GoBilda Super speed servo: 0.45
// - REV SRS: 1.1 (This is the max, if you've set smaller limits in hardware lower this number
class ServoMultiState(
    hardwareMap: HardwareMap,
    name: String,
    val states: DoubleArray,
    val fullRotationTime: Double,
    var currentPosition: Double = 0.0
) {
    val servo = hardwareMap.get(Servo::class.java, name)

    fun setManualLocation(newPosition: Double): Action {
        return object : Action {
            var initialized = false
            var sleepAction: Action? = null
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    initialized = true
                    if (currentPosition == newPosition) return false

                    val transitionTime =
                        abs(currentPosition - newPosition) * fullRotationTime

                    currentPosition = newPosition
                    servo.position = newPosition
                    sleepAction = SleepAction(transitionTime)
                }

                return sleepAction?.run(p) != false
            }
        }
    }

    fun setPosition(newState: Int): Action {
        return object : Action {
            var initialized = false
            var sleepAction: Action? = null
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    initialized = true
                    if (currentPosition == states[newState]) return false

                    val transitionTime =
                        abs(currentPosition - states[newState]) * fullRotationTime

                    currentPosition = states[newState]
                    servo.position = states[newState]
                    sleepAction = SleepAction(transitionTime)
                }

                return sleepAction?.run(p) != false
            }
        }
    }
}