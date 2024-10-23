package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit
import kotlin.math.abs

@Config
class Lift @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
    val ticksPerInch: Double = 47.7419354839,
) {
    companion object {
        @JvmField var tolerance = 0.5
    }

    @JvmField val liftMotor = hardwareMap.get(DcMotorEx::class.java, name)
    @JvmField var targetDistance: Double = 0.0

    val currentPosition get() = liftMotor.currentPosition / ticksPerInch

    private fun initMotor() {
        liftMotor.power = 0.0
        liftMotor.mode = DcMotor.RunMode.RESET_ENCODERS
        liftMotor.targetPosition = 0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.power = 1.0
    }

    init {
        initMotor()
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.setPositionPIDFCoefficients(pValue)
        liftMotor.direction = direction
    }

    fun resetPosition(resetCondition: Action): Action {
        return object : Action {
                var initialized = false
                var endpointTimeout = Deadline(2, TimeUnit.SECONDS)
                override fun run(p: TelemetryPacket): Boolean {
                    if (!initialized) {
                        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        liftMotor.power = -0.25
                        endpointTimeout.reset()
                        initialized = true
                    }

                    if (endpointTimeout.hasExpired() || !resetCondition.run(p)) {
                        initMotor()

                        // If there were any ignored goto distance calls make them happy now
                        gotoDistance(targetDistance)

                        return false
                    }
                    return true
                }
            }
    }

    fun gotoDistance(distance: Double): Action {
        targetDistance = distance

        return object : Action {
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    liftMotor.targetPosition = (targetDistance * ticksPerInch).toInt()
                    initialized = true
                }

                return abs(currentPosition - targetDistance) < tolerance
            }
        }
    }

    fun slideBetween(startPosition: Double, endPosition: Double, time: Double, units: TimeUnit = TimeUnit.SECONDS): Action {
        return object : Action {
            val startTime = System.nanoTime()
            override fun run(p: TelemetryPacket): Boolean {
                val currentTime = System.nanoTime()

                return abs(currentPosition - targetDistance) < tolerance
            }
        }
    }
}