package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout
import org.firstinspires.ftc.teamcode.galahlib.actions.doWhile
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ContinuousServo
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoToggle
import org.firstinspires.ftc.teamcode.messages.ColorMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import java.util.concurrent.TimeUnit
import kotlin.math.max

@Config
class Intake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField // Forward intake power
        var P_Intake: Double = 5.0

        @JvmField // Speed while intaking, if it is flying past lower this number
        var speed = 0.8

        @JvmField
        var transferSpeed = 0.8

        @JvmField
        var captureTimeout = 250L

        @JvmField
        var currentTrigger = 2.0

        @JvmField
        var currentTriggerSpeed = 0.4

        @JvmField
        var minExtension = 5.0

        @JvmField
        var maxExtension = 11.0

        @JvmField
        var extendTime = 5.0 // Over seconds

        class FlipLimits {
            @JvmField
            var downPosition = 0.36

            @JvmField
            var upPosition = 0.04
        }

        @JvmField
        val flipLimits = FlipLimits()
    }

    val slides = Lift(
        hardwareMap,
        "intakeSlides",
        DcMotorSimple.Direction.FORWARD,
        P_Intake,
        85.935483871
    )
    val leftEndpoint = DigitalInput(hardwareMap, "intakeLeft")
    val rightEndpoint = DigitalInput(hardwareMap, "intakeRight")

    val flipServo = ServoToggle(
        hardwareMap,
        "intakeFlip",
        flipLimits.upPosition,
        flipLimits.downPosition,
        1.5
    )
    val pullServo = ContinuousServo(
        hardwareMap,
        "intakeContinuous",
        DcMotorSimple.Direction.FORWARD
    )
    val motor = hardwareMap.get(DcMotorEx::class.java, "intake")
    val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColour")
    val distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "intakeDistance")

    init {
//        FlightRecorder.write("INTAKE_PARAMS", PARAMS.javaClass)
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    val sampleType: SampleType
        get() {
            val color = this.colorSensor.normalizedColors
            val max = max(color.red, max(color.green, color.blue))

            color.red /= max
            color.green /= max
            color.blue /= max

            Logging.DEBUG("Current colour", ColorMessage(color))

            return if (color.red == 1f) {
                SampleType.Red
            } else if (color.blue == 1f) {
                SampleType.Blue
            } else if (color.green == 1f) {
                SampleType.Shared
            } else {
                SampleType.Unknown
            }
        }
    val distanceTriggered: Boolean
        get() {
            return frontTriggered || backTriggered
        }
    val frontTriggered: Boolean
        get() {
            val distance = colorSensor.getDistance(DistanceUnit.MM)
            Logging.DEBUG("FRONT_DISTANCE", distance)
            return distance < 25
        }
    val backTriggered: Boolean
        get() {
            val distance = distanceSensor.getDistance(DistanceUnit.MM)
            Logging.DEBUG("REAR_DISTANCE", distance)
            return distance < 50
        }

    fun resetSlides(): LoggableAction {
        return slides.resetPosition(
            race(
                leftEndpoint.until(false),
                rightEndpoint.until(false)
            )
        )
    }

    fun extendSlides(): LoggableAction {
        return LoggingSequential(
            "EXTEND_INTAKE",
            slides.gotoDistance(minExtension),
            Loggable("FLIP_INTAKE_FINISH_DEPLOY", flipServo.setPosition(true))
        )
    }

    fun crawlForward(extendTime: Double = PARAMS.extendTime): LoggableAction {
        return Loggable(
            "CRAWL_FORWARD_INTAKE", ParallelAction(
                InstantAction {
                    Logging.LOG("CRAWL_FORWARD")
                },
                slides.slideBetween(minExtension, maxExtension, extendTime)
            )
        )
    }

    fun crawlBackward(): LoggableAction {
        return Loggable(
            "CRAWL_BACKWARD_INTAKE", ParallelAction(
                InstantAction {
                    Logging.LOG("CRAWL_BACKWARD")
                },
                slides.gotoDistance((maxExtension + minExtension) / 2)
            )
        )
    }

    fun pickSampleForward(): LoggableAction {
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
                    motor.power = -speed
                } else if (currentSample == allianceColor || currentSample == SampleType.Shared) { // Good Sample
                    if (backTriggered) {
                        if (captureTimeout == null)
                            captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                        motor.power = 0.0
                    } else if (motor.getCurrent(CurrentUnit.AMPS) < currentTrigger) {
                        motor.power = -currentTriggerSpeed
                    } else {
                        motor.power = -speed / 4
                    }
                } else { // Bad Sample
                    captureTimeout = null
                    motor.power = -1.0
                }

                return captureTimeout?.hasExpired() != true && !PoseStorage.shouldHallucinate
            }
        }
    }

    fun pickSampleBackward(): LoggableAction {
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
                    motor.power = speed
                } else if (currentSample == allianceColor || currentSample == SampleType.Shared || backTriggered) { // Any Sample
                    if (frontTriggered) {
                        if (captureTimeout == null)
                            captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                        motor.power = 0.0
                    } else if (motor.getCurrent(CurrentUnit.AMPS) < currentTrigger) {
                        Logging.LOG("INTAKE_CURRENT", motor.getCurrent(CurrentUnit.AMPS))
                        motor.power = currentTriggerSpeed
                    } else {
                        motor.power = speed / 4
                    }
                } else { // Bad Sample
                    captureTimeout = null
                    motor.power = 1.0
                }

                return captureTimeout?.hasExpired() != true && !PoseStorage.shouldHallucinate
            }
        }
    }

    fun retractSlides(): LoggableAction {
        return Loggable(
            "RETRACT_FLIP_INTAKE", ParallelAction(
                InstantAction { Logging.LOG("RETRACT_SLIDES") },
                Timeout(slides.gotoDistance(0.0, 0.1), 1500.0),
                SequentialAction(
                    InstantAction {
                        motor.power = 0.15
                    },
                    SleepAction(0.1),
                    InstantAction {
                        motor.power = 0.2
                    },
                    flipServo.setPosition(false),
                    SleepAction(0.5),
                    InstantAction {
                        motor.power = 0.0
                    },
                    transfer(),
                    stopTransfer(),
                    SleepAction(1.0),
                    pullServo.setSpeed(0.0),
                )
            )
        )
    }

    fun transfer(): LoggableAction {
        return object : LoggableAction {
            var initialized = false

            override val name: String
                get() = "INTAKE_TRANSFER"

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    Logging.LOG("TRANSFER")
                    motor.power = transferSpeed
                    initialized = true
                }

                return frontTriggered
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
    fun captureSample(forward: Boolean, close: Boolean = false): LoggableAction {
        if (close) return LoggingSequential(
            "CAPTURE_CLOSE_SAMPLE",

            Loggable(
                "SEARCH_AND_FLIP", ParallelAction(
                    Loggable("FLIP_INTAKE_DOWN", flipServo.setPosition(true)),
                    pullServo.setSpeed(1.0),
                    slides.gotoDistance(1.0),
                    if (forward) pickSampleForward() else pickSampleBackward(),
                )
            ),
            retractSlides(),
        )

        return LoggingSequential(
            "CAPTURE_FAR_SAMPLE",
            Loggable(
                "SEARCH_AND_FIND", ParallelAction(
                    pullServo.setSpeed(1.0),
                    extendSlides(),
                    doWhile(
                        if (forward) crawlForward() else crawlBackward(),
                        if (forward) pickSampleForward() else pickSampleBackward()
                    ),
                )
            ),
            retractSlides(),
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