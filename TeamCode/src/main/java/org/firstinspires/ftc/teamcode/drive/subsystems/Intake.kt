package org.firstinspires.ftc.teamcode.drive.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.drive.Logging
import org.firstinspires.ftc.teamcode.drive.PoseStorage
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.doWhile
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoToggle
import org.firstinspires.ftc.teamcode.messages.ColorMessage
import org.firstinspires.ftc.teamcode.messages.StringMessage
import java.util.concurrent.TimeUnit
import kotlin.math.max

@Config
class Intake(hardwareMap: HardwareMap) : StateLoggable {
    class Params {
        @JvmField var P_Intake: Double = 3.0

        @JvmField var speed = -0.53
        @JvmField var transferSpeed = 0.7
        @JvmField var captureTimeout = 750L

        @JvmField var currentTrigger = 0.5
        @JvmField var currentTriggerSpeed = -0.4

        @JvmField var minExtension = 5.0
        @JvmField var maxExtension = 11.0
        @JvmField var extendTime = 3.0 // Over seconds

        class FlipLimits {
            @JvmField var downPosition = 0.25
            @JvmField var upPosition = 0.03
        }
        @JvmField val flipLimits = FlipLimits()
    }

    companion object {
        val PARAMS = Params()
    }

    val slides = Lift(hardwareMap, "intakeSlides", DcMotorSimple.Direction.FORWARD, PARAMS.P_Intake, 85.935483871)
    val leftEndpoint = DigitalInput(hardwareMap, "intakeLeft")
    val rightEndpoint = DigitalInput(hardwareMap, "intakeRight")

    val flipServo = ServoToggle(hardwareMap, "intakeFlip", PARAMS.flipLimits.upPosition, PARAMS.flipLimits.downPosition, 1.17)
    val motor = hardwareMap.get(DcMotorEx::class.java, "intake")
    val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColour")
    val distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "intakeDistance")

    init {
        FlightRecorder.write("INTAKE_PARAMS", PARAMS)
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val sampleType: SampleType get() {
        val color = this.colorSensor.normalizedColors
        val max =  max(color.red, max(color.green, color.blue));

        color.red /= max;
        color.green /= max;
        color.blue /= max;

        Logging.DEBUG("Current colour", ColorMessage(color));

        return if (color.red == 1f) {
            SampleType.Red;
        } else if (color.blue == 1f) {
            SampleType.Blue;
        } else if (color.green == 1f) {
            SampleType.Shared;
        } else {
            SampleType.Unknown;
        }
    }
    val distanceTriggered: Boolean get() {
        val frontTriggered = colorSensor.getDistance(DistanceUnit.MM) < 25

        return frontTriggered || backTriggered
    }
    val backTriggered: Boolean get() {
        return distanceSensor.getDistance(DistanceUnit.MM) < 20
    }

    fun resetSlides(): LoggableAction {
        return slides.resetPosition(race(
                leftEndpoint.until(false),
                rightEndpoint.until(false)
            ))
    }

    fun extendSlides(): LoggableAction {
        return LoggingSequential(
            "EXTEND_INTAKE",
            slides.gotoDistance(PARAMS.minExtension / 2),
            Loggable("FLIP_INTAKE_FINISH_DEPLOY", ParallelAction(
                flipServo.setPosition(true),
                slides.gotoDistance(PARAMS.minExtension)
            ))
        )
    }

    fun crawlForward(extendTime: Double = PARAMS.extendTime): LoggableAction {
        return Loggable("CRAWL_FORWARD_INTAKE", ParallelAction(
            InstantAction {
                Logging.LOG("CRAWL_FORWARD")
            },
            slides.slideBetween(PARAMS.minExtension, PARAMS.maxExtension, extendTime)
        ))
    }

    fun pickSample(): LoggableAction {
        return object : LoggableAction {
            var captureTimeout: Deadline? = null
            override val name: String
                get() = "PICKING_SAMPLE_${if (captureTimeout == null) "SEARCHING" else "WAITING"}"
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    Logging.LOG("PICK_SAMPLE")
                    initialized = true
                }

                val currentSample = sampleType
                val allianceColor =
                    if (PoseStorage.isRedAlliance) SampleType.Red else SampleType.Blue

                if (!distanceTriggered) { // No Sample
                    captureTimeout = null
                    motor.power = PARAMS.speed
                } else if (currentSample == allianceColor || currentSample == SampleType.Shared) { // Good Sample
                    if (backTriggered) {
                        if (captureTimeout == null)
                            captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                        motor.power = 0.0
                    } else if (motor.getCurrent(CurrentUnit.AMPS) < PARAMS.currentTrigger) {
                        motor.power = PARAMS.currentTriggerSpeed
                    }
                } else { // Bad Sample
                    captureTimeout = null
                    motor.power = -1.0
                }

                return captureTimeout?.hasExpired() != true
            }
        }
    }

    fun retractSlides(): LoggableAction {
        return Loggable("RETRACT_FLIP_INTAKE", ParallelAction(
            InstantAction {Logging.LOG("RETRACT_SLIDES")},
            slides.gotoDistance(0.0),
            InstantAction {
                motor.power = 0.2
            },
            flipServo.setPosition(false)
        ))
    }

    fun transfer(): LoggableAction {
        return object : LoggableAction {
            var initialized = false

            override val name: String
                get() = "INTAKE_TRANSFER"
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    Logging.LOG("TRANSFER")
                    motor.power = PARAMS.transferSpeed
                    initialized = true
                }

                return backTriggered
            }
        }
    }

    fun stopTransfer(): LoggableAction {
        return Loggable("STOP_TRANSFER_MOTOR", InstantAction {
            Logging.LOG("STOP_TRANSFER")
            motor.power = 0.0
        })
    }

    @JvmOverloads
    fun captureSample(close: Boolean = false): LoggableAction {
        if (close) return LoggingSequential(
            "CAPTURE_CLOSE_SAMPLE",

            Loggable("FLIP_INTAKE_DOWN", flipServo.setPosition(true)),
            pickSample(),
            retractSlides(),
        )

        return LoggingSequential(
            "CAPTURE_FAR_SAMPLE",
            extendSlides(),
            doWhile(
                crawlForward(),
                pickSample()
            ),
            retractSlides()
        )
    }

    override fun logState(uniqueName: String?) {
        slides.logState("$uniqueName [INTAKE_SLIDES]")
        Logging.DEBUG("$uniqueName INTAKE_POWER", motor.power)
        Logging.DEBUG("$uniqueName INTAKE_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
    }

    fun lockout() {
        slides.lockout()
    }

    fun unlock() {
        slides.unlock()
    }
}