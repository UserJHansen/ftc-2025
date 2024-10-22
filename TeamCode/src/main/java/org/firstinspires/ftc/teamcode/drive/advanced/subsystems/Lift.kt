package org.firstinspires.ftc.teamcode.drive.advanced.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit

@Config
class Lift @JvmOverloads constructor(
    @JvmField
    val liftMotor: DcMotorEx,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
    val ticksPerInch: Double = 47.7419354839,
) {

    @JvmField
    var targetDistance: Double = 0.0
    private var endpointTimeout: Deadline? = null

    init {
        liftMotor.power = 0.0
        liftMotor.mode = DcMotor.RunMode.RESET_ENCODERS
        liftMotor.targetPosition = 0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = 1.0
        liftMotor.setPositionPIDFCoefficients(pValue)
        liftMotor.direction = direction
    }

    interface ResetCheck {
        fun call(): Boolean
    }

    private var endLimit: ResetCheck? = null
    fun reset(resetCondition: ResetCheck) {
        this.endLimit = resetCondition

        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor.power = -0.25
        endpointTimeout = Deadline(2, TimeUnit.SECONDS)
    }

    fun update() {
        endpointTimeout?.let {
            if (it.hasExpired() || this.endLimit?.call() == true) { // High is true, disconnected
                endpointTimeout = null
                liftMotor.power = 0.0
                liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                liftMotor.targetPosition = 0
                liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                liftMotor.power = 1.0
                setDistance(targetDistance)
            }
        }
    }

    fun currentPosition(): Double {
        return liftMotor.currentPosition / ticksPerInch
    }

    fun setDistance(distance: Double) {
        targetDistance = distance
        if (endpointTimeout == null) { // Still calibrating
            targetDistance = distance
            val target = (distance * ticksPerInch).toInt()
            liftMotor.targetPosition = target
        }
    }
}