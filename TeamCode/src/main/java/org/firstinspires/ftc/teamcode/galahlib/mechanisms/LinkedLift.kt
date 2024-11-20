package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.ftclib.PIDFController
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.messages.BooleanMessage
import org.firstinspires.ftc.teamcode.messages.DoubleMessage
import org.firstinspires.ftc.teamcode.messages.StringMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max

@Config
class LinkedLift @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    firstLiftName: String,
    secondLiftName: String,
    val pidfController: PIDFController,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    invertSecond: Boolean = true,
    val ticksPerInch: Double = 47.7419354839,
) : StateLoggable {
    companion object {
        @JvmField
        var tolerance = 0.5
    }

    @JvmField
    val firstLift = hardwareMap.get(DcMotorEx::class.java, firstLiftName)

    @JvmField
    val secondLift = hardwareMap.get(DcMotorEx::class.java, secondLiftName)

    @JvmField
    var targetDistance: Double = 0.0

    val currentPosition get() = (firstLift.currentPosition + secondLift.currentPosition) / 2 / ticksPerInch

    private fun initMotor(motor: DcMotorEx) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.power = 0.0
    }

    fun interface MotorFunction {
        fun run(motor: DcMotorEx)
    }

    fun forBothMotors(func: MotorFunction) {
        for (motor in arrayOf(firstLift, secondLift)) {
            func.run(motor)
        }
    }

    init {
        forBothMotors { motor ->
            initMotor(motor)
            motor.direction = direction
        }

        if (invertSecond) {
            secondLift.direction = direction.inverted()
        }
    }

    private val liftActionWriter = DownsampledWriter("LIFT_ACTION", 50_000_000)
    private val liftPoseWriter = DownsampledWriter("LIFT_POSITION", 50_000_000)

    fun resetPosition(resetCondition: Action): LoggableAction {
        liftActionWriter.write(StringMessage("$lastName RESET_POSITION_START"))
        return object : LoggableAction {
            var initialized = false
            var endpointTimeout = Deadline(2, TimeUnit.SECONDS)
            val resetAction = SequentialAction(
                resetCondition,
                InstantAction {
                    forBothMotors { motor ->
                        motor.power = 0.0
                    }
                },
                SleepAction(0.1)
            )
            override val name: String
                get() = "RESET_POSITION"

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    forBothMotors { motor ->
                        motor.power = -1.0
                    }
                    endpointTimeout.reset()
                    initialized = true
                }

                if (endpointTimeout.hasExpired() || !resetAction.run(p)) {
                    liftActionWriter.write(StringMessage("$lastName POSITION_RESET"))
                    FlightRecorder.write(
                        "$lastName RESET_TIMEOUT",
                        BooleanMessage(endpointTimeout.hasExpired())
                    )
                    liftPoseWriter.write(DoubleMessage(0.0))

                    forBothMotors { motor ->
                        motor.power = 0.0
                        motor.mode = DcMotor.RunMode.RESET_ENCODERS
                        initMotor(motor)
                    }

                    // If there were any ignored goto distance calls make them happy now
                    // However does not wait for it
                    gotoDistance(targetDistance).run(p)

                    return false
                }
                return true
            }
        }
    }

    @JvmOverloads
    fun gotoDistance(distance: Double, tolerance: Double = Companion.tolerance): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName GOTO_DISTANCE_$targetDistance"
            var initialized = false
            var timeout: Deadline? = null

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    targetDistance = max(distance, 0.1)
                    liftActionWriter.write(StringMessage("$lastName GOTO_DISTANCE"))

                    timeout = Deadline(
                        (0.75 * abs(targetDistance - currentPosition)).toLong(),
                        TimeUnit.SECONDS
                    )

                    initialized = true
                }

                liftPoseWriter.write(DoubleMessage(currentPosition))

                if (abs(currentPosition - targetDistance) > tolerance && timeout?.hasExpired() == false && !PoseStorage.shouldHallucinate) {
                    if (!lockedOut)
                        forBothMotors { motor ->
                            val power = pidfController.calculate(motor.currentPosition.toDouble())
                            motor.power = power
                        }

                    return true
                }

                if (abs(targetDistance - 0.0) < Companion.tolerance && abs(targetDistance - currentPosition) < Companion.tolerance) {
//                    Target is 0, and we're there, turn off the motor
                    forBothMotors { motor ->
                        motor.power = 0.0
                    }
                }
                return false
            }
        }
    }

    fun goToThroughWhile(
        distance: Double,
        throughPoint: Double,
        action: LoggableAction,
        tolerance: Double = Companion.tolerance
    ): LoggableAction {
        return object : LoggableAction {
            val goingUp = distance - throughPoint > 0
            val activateThrough: Boolean
                get() = (currentPosition < throughPoint) xor !goingUp

            override val name: String
                get() = "$lastName GO_TO_${distance}_THROUGH_${throughPoint}_${if (activateThrough) "ACTIVATED" else "MOVING"}"

            val gotoAction = gotoDistance(distance, tolerance)
            var throughCompleted = false

            override fun run(p: TelemetryPacket): Boolean {
                if (activateThrough && !throughCompleted) {
                    action.run(p)
                }

                return gotoAction.run(p)
            }
        }
    }

    var lockedOut = true
    fun lockout() {
        forBothMotors { motor ->
            motor.power = 0.0
        }
        lockedOut = true
    }

    fun unlock() {
        lockedOut = false
    }

    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName [FIRST] LIFT_POSITION", firstLift.currentPosition)
        Logging.DEBUG("$uniqueName [FIRST] LIFT_POWER", firstLift.power)
        Logging.DEBUG("$uniqueName [FIRST] LIFT_CURRENT", firstLift.getCurrent(CurrentUnit.AMPS))
        Logging.DEBUG("$uniqueName [SECOND] LIFT_POSITION", secondLift.currentPosition)
        Logging.DEBUG("$uniqueName [SECOND] LIFT_POWER", secondLift.power)
        Logging.DEBUG("$uniqueName [SECOND] LIFT_CURRENT", secondLift.getCurrent(CurrentUnit.AMPS))
    }
}