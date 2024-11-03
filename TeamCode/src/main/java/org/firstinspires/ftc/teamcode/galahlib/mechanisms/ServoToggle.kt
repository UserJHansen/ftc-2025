package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
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
class ServoToggle(
    hardwareMap: HardwareMap,
    name: String,
    val off: Double,
    val on: Double,
    fullRotationTime: Double
) {
    val servo = hardwareMap.get(Servo::class.java, name)
    val transitionTime = abs(on - off) * fullRotationTime
    var currentState = false

    fun setPosition(newState: Boolean): Action {
        return object : Action {
            var initialized = false
            var sleepAction: Action? = null
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    initialized = true
                    if (currentState == newState) return false

                    currentState = newState
                    servo.position = if (newState) on else off
                    sleepAction = SleepAction(transitionTime)
                }

                return sleepAction?.run(p) != false
            }
        }
        return SequentialAction(InstantAction {
            currentState = newState
            servo.position = if (newState) on else off
        }, SleepAction(transitionTime))
    }
}