package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap

class DigitalInput(hardwareMap: HardwareMap, name: String) {
    val input = hardwareMap.get(DigitalChannel::class.java, name)
    val triggered = input.state

    fun until(state: Boolean): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                return input.state != state
            }
        }
    }
}