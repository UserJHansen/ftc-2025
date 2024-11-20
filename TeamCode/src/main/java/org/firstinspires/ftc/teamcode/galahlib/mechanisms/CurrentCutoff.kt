package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.staticData.PoseStorage

class CurrentCutoff(val motor: DcMotorEx) {
    @JvmOverloads
    fun above(amount: Double, units: CurrentUnit = CurrentUnit.AMPS): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                return motor.getCurrent(units) < amount && !PoseStorage.shouldHallucinate
            }
        }
    }

    @JvmOverloads
    fun below(amount: Double, units: CurrentUnit = CurrentUnit.AMPS): Action {
        return object : Action {
            override fun run(p: TelemetryPacket): Boolean {
                return motor.getCurrent(units) > amount && !PoseStorage.shouldHallucinate
            }
        }
    }
}