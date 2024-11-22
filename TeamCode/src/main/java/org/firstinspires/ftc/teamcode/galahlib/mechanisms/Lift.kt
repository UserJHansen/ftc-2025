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
class Lift @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
    val ticksPerInch: Double = 47.7419354839,
) : StateLoggable {
    companion object {
        @JvmField
        var tolerance = 0.5
    }

    @JvmField
    val liftMotor = hardwareMap.get(DcMotorEx::class.java, name)

    @JvmField
    var targetDistance: Double = 0.0

    val currentPosition get() = liftMotor.currentPosition / ticksPerInch

    private fun initMotor() {
        liftMotor.targetPosition = 0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = 0.1
        liftMotor.targetPositionTolerance = (tolerance * ticksPerInch).toInt()
    }

    init {
        initMotor()
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.setPositionPIDFCoefficients(pValue)
        liftMotor.direction = direction
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
                    liftMotor.power = 0.0
                },
                SleepAction(0.1)
            )
            override val name: String
                get() = "RESET_POSITION"

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    liftMotor.power = -1.0
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

                    liftMotor.power = 0.0
                    liftMotor.mode = DcMotor.RunMode.RESET_ENCODERS
                    initMotor()

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
                    liftMotor.targetPosition = (targetDistance * ticksPerInch).toInt()
                    if (!lockedOut) {
                        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        liftMotor.power = 1.0
                    }

                    initialized = true
                }

                liftPoseWriter.write(DoubleMessage(currentPosition))

                if (abs(currentPosition - targetDistance) > tolerance && timeout?.hasExpired() == false && !PoseStorage.shouldHallucinate) {
                    return true
                }

                if (abs(targetDistance - 0.0) < Companion.tolerance && abs(targetDistance - currentPosition) < Companion.tolerance) {
//                    Target is 0, and we're there, turn off the motor
                    liftMotor.power = 0.0
                    liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                }
                return false
            }
        }
    }


    fun interface DoubleProvider {
        fun run(): Double
    }

    fun holdVariablePosition(positionProvider: DoubleProvider): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName HOLD_USER_DISTANCE"
            var initialized = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    liftActionWriter.write(StringMessage("$lastName HOLD_USER_DISTANCE"))
                    liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    liftMotor.power = 1.0
                    initialized = true
                }

                liftPoseWriter.write(DoubleMessage(currentPosition))

                if (!lockedOut) {
                    val position = positionProvider.run()
                    Logging.LOG("HOLDING_POSITION", position)
                    liftMotor.targetPosition = (position * ticksPerInch).toInt()
                }

                return liftMotor.targetPosition != 0
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
                get() = (currentPosition < throughPoint) xor goingUp

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
        liftMotor.power = 0.0
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lockedOut = true
    }

    fun unlock() {
        if (abs(targetDistance - 0.0) > tolerance) {
            liftMotor.power = 1.0
            liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        lockedOut = false
    }

    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName LIFT_POSITION", liftMotor.currentPosition)
        Logging.DEBUG("$uniqueName LIFT_TARGET", liftMotor.targetPosition)
        Logging.DEBUG("$uniqueName LIFT_MODE", liftMotor.mode)
        Logging.DEBUG("$uniqueName LIFT_POWER", liftMotor.power)
        Logging.DEBUG("$uniqueName LIFT_CURRENT", liftMotor.getCurrent(CurrentUnit.AMPS))
    }
}