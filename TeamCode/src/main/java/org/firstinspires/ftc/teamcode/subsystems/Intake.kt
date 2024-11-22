package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
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
        var P_Intake: Double = 15.0

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
        var minExtension = 3.0

        @JvmField
        var maxExtension = 11.0

        class FlipLimits {
            @JvmField
            var downPosition = 0.36

            @JvmField
            var upPosition = 0.06
        }

        @JvmField
        val flipLimits = FlipLimits()
    }

    val slides = Lift(
        hardwareMap,
        "intakeSlides",
        DcMotorSimple.Direction.FORWARD,
        P_Intake,
        68.47911742
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

    fun pickSampleForward(shared: Boolean): LoggableAction {
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
                } else if (currentSample == allianceColor || (currentSample == SampleType.Shared && shared)) { // Good Sample
                    if (backTriggered) {
                        if (captureTimeout == null)
                            captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                        motor.power = 0.0
                    StaticLights.colors[1] = when (currentSample) {
                        SampleType.Red -> RevBlinkinLedDriver.BlinkinPattern.RED
                        SampleType.Blue -> RevBlinkinLedDriver.BlinkinPattern.BLUE
                        SampleType.Shared -> RevBlinkinLedDriver.BlinkinPattern.YELLOW
                        else -> RevBlinkinLedDriver.BlinkinPattern.BLACK
                    }
                    } else if (motor.getCurrent(CurrentUnit.AMPS) < currentTrigger) {
                        motor.power = -currentTriggerSpeed
                    } else {
                        motor.power = -speed / 4
                    }
                } else { // Bad Sample
                    captureTimeout = null
                    motor.power = -1.0
                    StaticLights.colors[1] = RevBlinkinLedDriver.BlinkinPattern.BLACK
                }

                return captureTimeout?.hasExpired() != true && !PoseStorage.shouldHallucinate
            }
        }
    }

    fun cancelSlides(): LoggableAction {
        return Loggable("CANCEL_INTAKE", ParallelAction(
            slides.gotoDistance(0.0, 0.1),
            pullServo.setSpeed(0.0),
            flipServo.setPosition(false),
            InstantAction {
                motor.power = 0.0
            },
            StaticLights.setColours(arrayOf(
                RevBlinkinLedDriver.BlinkinPattern.BLACK,
            )),
        ))
    }

    fun retractSlides(): LoggableAction {
        return Loggable(
            "RETRACT_FLIP_INTAKE", ParallelAction(
                InstantAction { Logging.LOG("RETRACT_SLIDES") },
                slides.gotoDistance(0.0, 0.1),
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
        var initialized = false
        return Loggable("INTAKE_TRANSFER", fun(p: TelemetryPacket): Boolean {
            if (!initialized) {
                Logging.LOG("TRANSFER")
                motor.power = transferSpeed
                pullServo.setSpeed(1.0).run(p)
                initialized = true
            }

            return frontTriggered
        })
    }

    fun stopTransfer(): LoggableAction {
        return Loggable("STOP_TRANSFER_MOTOR", InstantAction {
            Logging.LOG("STOP_TRANSFER")
            motor.power = 0.0
            pullServo.servo.power = 0.0
        })
    }

    @JvmOverloads
    fun captureSample(
        shared: Boolean,
        positionProvider: Lift.DoubleProvider? = null
    ): LoggableAction {
        if (positionProvider?.run() == null) return LoggingSequential(
            "CAPTURE_CLOSE_SAMPLE",

            StaticLights.setColours(arrayOf(
                RevBlinkinLedDriver.BlinkinPattern.WHITE,
                RevBlinkinLedDriver.BlinkinPattern.BLACK,
            )),
            Loggable(
                "SEARCH_AND_FLIP", ParallelAction(
                    Loggable("FLIP_INTAKE_DOWN", flipServo.setPosition(true)),
                    pullServo.setSpeed(1.0),
                    slides.gotoDistance(1.0),
                    pickSampleForward(shared),
                )
            ),
        )

        val holdPositionAction = slides.holdVariablePosition(positionProvider)
        val flipIntakeAction = flipServo.setPosition(true)
        var flipComplete = false

        return LoggingSequential(
            "CAPTURE_FAR_SAMPLE",
            StaticLights.setColours(arrayOf(
                RevBlinkinLedDriver.BlinkinPattern.WHITE,
                RevBlinkinLedDriver.BlinkinPattern.BLACK,
            )),
            Loggable(
                "SEARCH_AND_FIND", ParallelAction(
                    pullServo.setSpeed(1.0),
                    race(
                        fun(p: TelemetryPacket): Boolean {
                            if (!flipComplete && slides.currentPosition > minExtension) {
                                flipComplete = !flipIntakeAction.run(p)
                            }

                            return holdPositionAction.run(p)
                        },
                        pickSampleForward(shared)
                    ),
                )
            ),
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
