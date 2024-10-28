package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class DistanceTrigger(val sensor: DistanceSensor) {
    @JvmOverloads
    fun closerThan(distance: Double, units: DistanceUnit = DistanceUnit.MM): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                return sensor.getDistance(units) > distance
            }
        }
    }

    @JvmOverloads
    fun furtherThan(distance: Double, units: DistanceUnit = DistanceUnit.MM): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                return sensor.getDistance(units) < distance
            }
        }
    }
}