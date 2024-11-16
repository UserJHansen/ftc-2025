package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.roadrunner.InstantAction
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction

class ContinuousServo(
    hardwareMap: HardwareMap,
    val name: String,
    direction: DcMotorSimple.Direction
) {
    val servo = hardwareMap.get(CRServo::class.java, name)

    init {
        servo.direction = direction
    }

    fun setSpeed(speed: Double): LoggableAction {
        return Loggable("SET_${name}_SPEED", InstantAction {
            servo.power = speed
        })
    }
}