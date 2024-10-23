package org.firstinspires.ftc.teamcode.drive.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DistanceTrigger
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState

class Outtake(hardwareMap: HardwareMap) {
    companion object PARAMS {
        @JvmField var P_Outake = 3.0
        object liftPositions {
            @JvmField var TopBasket = 27.0
            @JvmField var TopSpecimen = 20.0
        }

        object grabberLimits {
            @JvmField var waiting = 0.5
            @JvmField var grabbing = 1.0
        }
        object elbowLimits {
            @JvmField var intake = 0.15
            @JvmField var deposit = 1.0
            @JvmField var specimen = 0.48
            @JvmField var transfer = 0.0
        }
        object wristLimits {
            @JvmField var intake = 0.33
            @JvmField var deposit = 0.15
            @JvmField var specimen = 0.15
            @JvmField var transfer = 0.4
        }
    }

    val lift = Lift(hardwareMap, "outtakeSlides", DcMotorSimple.Direction.REVERSE, P_Outake)
    val endLimit = DigitalInput(hardwareMap, "outtakeLimit")
    val transferCradle = hardwareMap.get(RevColorSensorV3::class.java, "transferCradle")

    val grabber = ServoMultiState(hardwareMap, "outtakeGrabber",
        doubleArrayOf(grabberLimits.waiting, grabberLimits.grabbing), 1.17)
    val elbow = ServoMultiState(hardwareMap, "elbow",
        doubleArrayOf(elbowLimits.intake, elbowLimits.deposit, elbowLimits.specimen, elbowLimits.transfer), 1.17)
    val wrist = ServoMultiState(hardwareMap, "wrist",
        doubleArrayOf(wristLimits.intake, wristLimits.deposit, wristLimits.specimen, wristLimits.transfer), 1.17)

    init {
        FlightRecorder.write("OUTTAKE_PARAMS", PARAMS)
    }

    fun resetLifts(): Action {
        return lift.resetPosition(endLimit.until(false))
    }

    fun readyForTransfer(): Action {
        return ParallelAction(
            elbow.setPosition(3),
            wrist.setPosition(3)
        )
    }

    fun waitForTransfer(): Action {
        return DistanceTrigger(transferCradle).closerThan(40.0)
    }

    fun pickupInternalSample(): Action {
        return SequentialAction(
            ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            ),
            grabber.setPosition(1)
        )
    }

    fun topBasket(): Action {
        return SequentialAction(
            lift.gotoDistance(liftPositions.TopBasket/2),
            ParallelAction(
                elbow.setPosition(1),
                wrist.setPosition(1),
                lift.gotoDistance(liftPositions.TopBasket)
            ),
        )
    }

    fun dropSample(): Action {
        return SequentialAction(
            grabber.setPosition(0),
            SleepAction(3.0),
            ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            ),
            lift.gotoDistance(0.0)
        )
    }

    fun specimenReady(): Action {
        return SequentialAction(
            lift.gotoDistance(3.0),
            ParallelAction(
                elbow.setPosition(2),
                wrist.setPosition(2)
            ),
            lift.gotoDistance(0.1),
        )
    }

    fun grabSpecimen(): Action {
        return SequentialAction(
            grabber.setPosition(1),
            SleepAction(2.0),
            lift.gotoDistance(liftPositions.TopSpecimen)
        )
    }

    fun placeSpecimen(): Action {
        return SequentialAction(
            lift.gotoDistance(liftPositions.TopSpecimen-2),
            SleepAction(2.0),
            grabber.setPosition(0),
            ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            ),
            lift.gotoDistance(0.0)
        )
    }

    fun abortSpecimen(): Action {
        return SequentialAction(
            lift.gotoDistance(3.0),
            ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            ),
            lift.gotoDistance(0.0),
        )
    }
}