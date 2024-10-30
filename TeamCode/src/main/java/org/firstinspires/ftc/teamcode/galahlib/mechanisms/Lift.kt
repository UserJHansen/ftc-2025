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
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.messages.BooleanMessage
import org.firstinspires.ftc.teamcode.messages.DoubleMessage
import org.firstinspires.ftc.teamcode.messages.StringMessage
import java.util.concurrent.TimeUnit
import kotlin.math.abs

@Config
class Lift @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    pValue: Double = 1.0,
    val ticksPerInch: Double = 47.7419354839,
) : StateLoggable {
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
                        FlightRecorder.write("$lastName RESET_TIMEOUT", BooleanMessage(endpointTimeout.hasExpired()))
                        liftPoseWriter.write(DoubleMessage(0.0))

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

    fun gotoDistance(distance: Double): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = "$lastName GOTO_DISTANCE_$targetDistance"
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    targetDistance = if (distance != 0.0) distance else tolerance
                    liftActionWriter.write(StringMessage("$lastName GOTO_DISTANCE"))

                    liftMotor.targetPosition = (targetDistance * ticksPerInch).toInt()
                    initialized = true
                }

                liftPoseWriter.write(DoubleMessage(currentPosition))
                return abs(currentPosition - targetDistance) > tolerance
            }
        }
    }

    fun slideBetween(startPosition: Double, endPosition: Double, time: Double, units: TimeUnit = TimeUnit.SECONDS): Action {
        return object : Action {
            val startTime = units.convert(System.nanoTime(), TimeUnit.NANOSECONDS)
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    liftActionWriter.write(StringMessage("$lastName SLIDING_BETWEEN"))
                    initialized = true
                }

                val currentTime = units.convert(System.nanoTime(), TimeUnit.NANOSECONDS)

                liftMotor.targetPosition = ((((endPosition-startPosition) * (currentTime-startTime)/time) + startPosition) * ticksPerInch).toInt()
                liftPoseWriter.write(DoubleMessage(currentPosition))

                return currentTime-startTime < time
            }
        }
    }

    fun lockout() {
        liftMotor.power = 0.0
    }

    fun unlock() {
        liftMotor.power = 1.0
    }

    var lastName = ""
    override fun logState(uniqueName: String) {
        lastName = uniqueName
        Logging.DEBUG("$uniqueName LIFT_POSITION", liftMotor.currentPosition)
        Logging.DEBUG("$uniqueName LIFT_TARGET", liftMotor.targetPosition)
        Logging.DEBUG("$uniqueName LIFT_CURRENT", liftMotor.getCurrent(CurrentUnit.AMPS))
    }
}