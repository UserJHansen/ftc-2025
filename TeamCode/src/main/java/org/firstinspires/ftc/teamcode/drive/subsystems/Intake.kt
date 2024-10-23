package org.firstinspires.ftc.teamcode.drive.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.drive.Logging
import org.firstinspires.ftc.teamcode.drive.PoseStorage
import org.firstinspires.ftc.teamcode.galahlib.actions.doWhile
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoToggle
import java.util.concurrent.TimeUnit
import kotlin.math.max

@Config
class Intake(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField var P_Intake: Double = 6.0

        @JvmField var speed = -0.53
        @JvmField var transferSpeed = -0.4
        @JvmField var captureTimeout = 500L

        @JvmField var minExtension = 5.0
        @JvmField var maxExtension = 11.0
        @JvmField var extendTime = 3.0 // Over seconds

        object FlipLimits {
            @JvmField var downPosition = 0.35
            @JvmField var upPosition = 0.05
        }
    }

    val slides = Lift(hardwareMap, "intakeSlides", DcMotorSimple.Direction.REVERSE, PARAMS.P_Intake)
    val leftEndpoint = DigitalInput(hardwareMap, "intakeLeft")
    val rightEndpoint = DigitalInput(hardwareMap, "intakeRight")

    val flipServo = ServoToggle(hardwareMap, "intakeFlip", PARAMS.FlipLimits.downPosition, PARAMS.FlipLimits.upPosition, 1.17)
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

        Logging.DEBUG("Current colour", "${color.red} ${color.green} ${color.blue} ${color.alpha}");

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
        val backTriggered = distanceSensor.getDistance(DistanceUnit.MM) < 60

        return frontTriggered || backTriggered
    }

    fun resetSlides(): Action {
        return slides.resetPosition(race(
                leftEndpoint.until(false),
                rightEndpoint.until(false)
            ))
    }

    fun extendSlides(): Action {
        return SequentialAction(
            slides.gotoDistance(PARAMS.minExtension / 2),
            ParallelAction(
                flipServo.setPosition(true),
                slides.gotoDistance(PARAMS.minExtension)
            )
        )
    }

    fun crawlForward(extendTime: Double = PARAMS.extendTime): Action {
        return slides.slideBetween(PARAMS.minExtension, PARAMS.maxExtension, extendTime)
    }

    fun pickSample(): Action {
        return object : Action {
            var captureTimeout: Deadline? = null
            override fun run(p: TelemetryPacket): Boolean {
                val currentSample = sampleType
                val allianceColor =
                    if (PoseStorage.isRedAlliance) SampleType.Red else SampleType.Blue

                if (!distanceTriggered) { // No Sample
                    captureTimeout = null
                    motor.power = PARAMS.speed
                } else if (currentSample == allianceColor || currentSample == SampleType.Shared) { // Good Sample
                    captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                    motor.power = 0.0
                } else { // Bad Sample
                    captureTimeout = null
                    motor.power = -1.0
                }

                return captureTimeout?.hasExpired() != true
            }
        }
    }

    fun retractSlides(): Action {
        return ParallelAction(
            slides.gotoDistance(0.0),
            flipServo.setPosition(false)
        )
    }

    fun transfer(): Action {
        return object : Action {
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    motor.power = transferSpeed
                    initialized = true
                }

                return distanceTriggered
            }
        }
    }

    fun stopTransfer(): Action {
        return InstantAction({
            motor.power = 0.0
        })
    }

    @JvmOverloads
    fun captureSample(close: Boolean = false): Action {
        if (close) return SequentialAction(
            flipServo.setPosition(true),
            pickSample(),
            retractSlides(),
        )

        return SequentialAction(
            extendSlides(),
            doWhile(
                crawlForward(),
                pickSample()
            ),
            retractSlides()
        )
    }
}