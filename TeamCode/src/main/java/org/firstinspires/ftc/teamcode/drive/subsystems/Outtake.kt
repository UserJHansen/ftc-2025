package org.firstinspires.ftc.teamcode.drive.subsystems

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.drive.Logging
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DistanceTrigger
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState
import org.firstinspires.ftc.teamcode.messages.StringMessage

class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    class Params {
        @JvmField var P_Outake = 3.0
        class LiftPositions {
            @JvmField var topBasket = 27.0
            @JvmField var topSpecimen = 20.0
        }

        class GrabberLimits {
            @JvmField var waiting = 0.5
            @JvmField var grabbing = 1.0
        }
        class ElbowLimits {
            @JvmField var intake = 0.19
            @JvmField var deposit = 0.92
            @JvmField var specimen = 0.45
            @JvmField var transfer = 0.03
        }
        class WristLimits {
            @JvmField var intake = 0.57
            @JvmField var deposit = 0.42
            @JvmField var specimen = 0.38
            @JvmField var transfer = 0.67
        }

        val liftPositions = LiftPositions()
        val grabberLimits = GrabberLimits()
        val elbowLimits = ElbowLimits()
        val wristLimits = WristLimits()
    }

    companion object {
        @JvmField val PARAMS = Params()
    }
    val lift = Lift(hardwareMap, "outtakeSlides", DcMotorSimple.Direction.FORWARD, PARAMS.P_Outake)
    val endLimit = DigitalInput(hardwareMap, "outtakeLimit")
    val transferCradle = hardwareMap.get(RevColorSensorV3::class.java, "transferCradle")

    val grabber = ServoMultiState(hardwareMap, "outtakeGrabber",
        doubleArrayOf(PARAMS.grabberLimits.waiting, PARAMS.grabberLimits.grabbing), 1.17)
    val elbow = ServoMultiState(hardwareMap, "elbow",
        doubleArrayOf(PARAMS.elbowLimits.intake, PARAMS.elbowLimits.deposit, PARAMS.elbowLimits.specimen, PARAMS.elbowLimits.transfer), 1.17)
    val wrist = ServoMultiState(hardwareMap, "wrist",
        doubleArrayOf(PARAMS.wristLimits.intake, PARAMS.wristLimits.deposit, PARAMS.wristLimits.specimen, PARAMS.wristLimits.transfer), 1.17)

    init {
        FlightRecorder.write("OUTTAKE_PARAMS", PARAMS)
    }
    private val outtakeActionWriter = DownsampledWriter("OUTTAKE_ACTION", 50_000_000)

    fun resetLifts(): LoggableAction {
        outtakeActionWriter.write(StringMessage("RESET_LIFTS"))
        return lift.resetPosition(endLimit.until(false))
    }

    fun readyForTransfer(): LoggableAction {
        outtakeActionWriter.write(StringMessage("MOVE_ARM_TRANSFER"))
        return Loggable("MOVE_ARM_TRANSFER", ParallelAction(
            elbow.setPosition(3),
            wrist.setPosition(3)
        ))
    }

    fun waitForTransfer(): LoggableAction {
        outtakeActionWriter.write(StringMessage("WAIT_FOR_TRANSFER"))
        return Loggable("WAIT_DETECT_TRANSFER", DistanceTrigger(transferCradle).closerThan(40.0))
    }

    fun pickupInternalSample(): LoggableAction {
        outtakeActionWriter.write(StringMessage("PICKUP_INTERNAL_SAMPLE"))
        return LoggingSequential(
            "PICKUP_INTERNAL_SAMPLE",
            Loggable("MOVE_DOWN_POSITION", ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            )),
            Loggable("GRAB_SAMPLE", grabber.setPosition(1))
        )
    }

    fun topBasket(): LoggableAction {
        outtakeActionWriter.write(StringMessage("TOP_BASKET"))
        return LoggingSequential(
            "GOTO_TOP_BASKET",
            lift.gotoDistance(PARAMS.liftPositions.topBasket/2),
            Loggable("EXTEND_ARM_AND_GOTO_TOP",
                ParallelAction(
                    elbow.setPosition(1),
                    wrist.setPosition(1),
                    lift.gotoDistance(PARAMS.liftPositions.topBasket)
            )),
        )
    }

    fun dropSample(): LoggableAction {
        outtakeActionWriter.write(StringMessage("DROP_SAMPLE"))
        return LoggingSequential(
            "DROP_SAMPLE",
            Loggable("RELEASE_SAMPLE", grabber.setPosition(0)),
            Loggable("WAIT_FOR_ESCAPE", SleepAction(3.0)),
            Loggable("MOVE_ARM_IN", ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            )),
            lift.gotoDistance(0.0)
        )
    }

    fun specimenReady(): LoggableAction {
        outtakeActionWriter.write(StringMessage("SPECIMEN_READY"))
        return LoggingSequential(
            "SPECIMEN_READY",
            lift.gotoDistance(3.0),
            Loggable("MOVE_ARM_OUT", ParallelAction(
                elbow.setPosition(2),
                wrist.setPosition(2)
            )),
            lift.gotoDistance(0.1),
        )
    }

    fun grabSpecimen(): LoggableAction {
        outtakeActionWriter.write(StringMessage("GRAB_SPECIMEN"))
        return LoggingSequential(
            "GRAB_SPECIMEN",
            Loggable("GRAB_SPECIMEN", grabber.setPosition(1)),
            lift.gotoDistance(3.0),
            Loggable("WAIT_FOR_ESCAPE", SleepAction(2.0)),
            lift.gotoDistance(PARAMS.liftPositions.topSpecimen)
        )
    }

    fun placeSpecimen(): LoggableAction {
        outtakeActionWriter.write(StringMessage("PLACE_SPECIMEN"))
        return LoggingSequential(
            "PLACE_SPECIMEN",
            lift.gotoDistance(PARAMS.liftPositions.topSpecimen-2),
            Loggable("HOPE_LIFTS_CATCH_UP_AND_RELEASED", SleepAction(2.0)),
            Loggable("LET_GO", grabber.setPosition(0)),
            Loggable("MOVE_ARM_IN", ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            )),
            lift.gotoDistance(0.0)
        )
    }

    fun abortSpecimen(): LoggableAction {
        outtakeActionWriter.write(StringMessage("ABORT_SPECIMEN"))
        return LoggingSequential(
            "ABORT_SPECIMEN",
            lift.gotoDistance(3.0),
            Loggable("MOVE_ARM_IN", ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            )),
            lift.gotoDistance(0.0),
        )
    }

    override fun logState(uniqueName: String?) {
        lift.logState("$uniqueName [OUTTAKE_SLIDES]")
    }
}