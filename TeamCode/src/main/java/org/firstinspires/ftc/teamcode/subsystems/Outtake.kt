package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DistanceTrigger
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState
import org.firstinspires.ftc.teamcode.messages.StringMessage

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    class Params {
        @JvmField
        var P_Outake = 3.0

        class LiftPositions {
            @JvmField
            var topBasket = 27.0

            @JvmField
            var topSpecimen = 14.5
        }

        class GrabberLimits {
            @JvmField
            var waiting = 0.5

            @JvmField
            var grabbing = 1.0
        }

        class ElbowLimits {
            @JvmField
            var intake = 0.17

            @JvmField
            var deposit = 0.92

            @JvmField
            var specimen = 0.45

            @JvmField
            var transfer = 0.02
        }

        class WristLimits {
            @JvmField
            var intake = 0.51

            @JvmField
            var deposit = 0.35

            @JvmField
            var specimen = 0.3

            @JvmField
            var transfer = 0.57

            @JvmField
            var transferSafe = 0.35
        }

        val liftPositions = LiftPositions()
        val grabberLimits = GrabberLimits()
        val elbowLimits = ElbowLimits()
        val wristLimits = WristLimits()
    }

    companion object {
        @JvmStatic val PARAMS = Params()
    }

    val lift = Lift(hardwareMap, "outtakeSlides", DcMotorSimple.Direction.FORWARD, PARAMS.P_Outake)
    val endLimit = DigitalInput(hardwareMap, "outtakeLimit")
    val transferCradle = hardwareMap.get(RevColorSensorV3::class.java, "transferCradle")

    val grabber = ServoMultiState(
        hardwareMap, "outtakeGrabber",
        doubleArrayOf(PARAMS.grabberLimits.waiting, PARAMS.grabberLimits.grabbing), 1.17
    )
    val elbow = ServoMultiState(
        hardwareMap, "elbow",
        doubleArrayOf(
            PARAMS.elbowLimits.intake,
            PARAMS.elbowLimits.deposit,
            PARAMS.elbowLimits.specimen,
            PARAMS.elbowLimits.transfer
        ), 1.17
    )
    val wrist = ServoMultiState(
        hardwareMap, "wrist",
        doubleArrayOf(
            PARAMS.wristLimits.intake,
            PARAMS.wristLimits.deposit,
            PARAMS.wristLimits.specimen,
            PARAMS.wristLimits.transfer,
            PARAMS.wristLimits.transferSafe,
        ), 1.17
    )

    init {
        FlightRecorder.write("OUTTAKE_PARAMS", PARAMS)
    }

    private val outtakeActionWriter = DownsampledWriter("OUTTAKE_ACTION", 50_000_000)

    fun resetLifts(): LoggableAction {
        outtakeActionWriter.write(StringMessage("RESET_LIFTS"))
        return lift.resetPosition(endLimit.until(false))
    }

    fun readyForTransfer(): LoggableAction {
        return Loggable(
            "MOVE_ARM_TRANSFER", SequentialAction(
                InstantAction {
                    outtakeActionWriter.write(StringMessage("MOVE_ARM_TRANSFER"))
                },
                wrist.setPosition(3),
                elbow.setPosition(3),
                SleepAction(0.3)
            )
        )
    }

    fun waitForTransfer(): LoggableAction {
        return Loggable("WAIT_DETECT_TRANSFER", ParallelAction(InstantAction {
            outtakeActionWriter.write(StringMessage("WAIT_FOR_TRANSFER"))
        }, DistanceTrigger(transferCradle).closerThan(20.0)))
    }

    fun pickupInternalSample(): LoggableAction {
        return LoggingSequential(
            "PICKUP_INTERNAL_SAMPLE",
            Loggable(
                "MOVE_DOWN_POSITION", SequentialAction(
                    InstantAction {
                        outtakeActionWriter.write(StringMessage("PICKUP_INTERNAL_SAMPLE"))
                    },
                    race(
                        SequentialAction(
                            grabber.setPosition(1),
                            ParallelAction(
                                elbow.setPosition(0),
                                wrist.setPosition(0),
                                SleepAction(0.1)
                            ),
                            readyForTransfer(),
                            ParallelAction(
                                elbow.setPosition(0),
                                wrist.setPosition(0),
                                SleepAction(0.1)
                            ),
                            readyForTransfer(),
                            ParallelAction(
                                elbow.setPosition(0),
                                wrist.setPosition(0),
                                SleepAction(0.1)
                            ),
                            readyForTransfer(),
                        ),
                        waitForTransfer()
                    ),
                    grabber.setPosition(0),
                    SequentialAction(
                        elbow.setPosition(0),
                        wrist.setPosition(4),
                        SleepAction(0.25),
                        wrist.setPosition(0),
                        SleepAction(0.25),
                    ),
                )
            ),
            Loggable("GRAB_SAMPLE", grabber.setPosition(1))
        )
    }

    fun topBasket(): LoggableAction {
        return LoggingSequential(
            "GOTO_TOP_BASKET",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("TOP_BASKET"))
            }),
            lift.gotoDistance(PARAMS.liftPositions.topBasket / 2),
            Loggable(
                "EXTEND_ARM_AND_GOTO_TOP",
                ParallelAction(
                    elbow.setPosition(1),
                    wrist.setPosition(1),
                    lift.gotoDistance(PARAMS.liftPositions.topBasket)
                )
            ),
        )
    }

    fun dropSample(): LoggableAction {
        return LoggingSequential(
            "DROP_SAMPLE",
            Loggable("RELEASE_SAMPLE", ParallelAction(InstantAction {
                outtakeActionWriter.write(StringMessage("DROP_SAMPLE"))
            }, grabber.setPosition(0))),
        )
    }

    fun retractArm(): LoggableAction = LoggingSequential(
        "RETRACT_ARM",
        Loggable("LOG", InstantAction {
            outtakeActionWriter.write(StringMessage("DROP_SAMPLE"))
        }),
        Loggable(
            "MOVE_ARM_IN", ParallelAction(
                elbow.setPosition(0),
                wrist.setPosition(0)
            )
        ),
        lift.gotoDistance(0.0)
    )

    fun specimenReady(): LoggableAction {
        return LoggingSequential(
            "SPECIMEN_READY",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("SPECIMEN_READY"))
            }),
            lift.gotoDistance(3.0),
            Loggable(
                "MOVE_ARM_OUT", ParallelAction(
                    elbow.setPosition(2),
                    wrist.setPosition(2)
                )
            ),
            lift.gotoDistance(0.1),
        )
    }

    fun retryGrabSpecimen(): LoggableAction {
        return LoggingSequential(
            "GRAB_SPECIMEN_RETRY",
            Loggable("MOVE_GRABBER", ParallelAction(InstantAction {
                outtakeActionWriter.write(StringMessage("GRAB_SPECIMEN_RETRY"))
            }, grabber.setPosition(0))),
            lift.gotoDistance(0.1),
            Loggable("WAIT_FOR_REALIGN", SleepAction(1.0)),
        )
    }

    fun grabSpecimen(): LoggableAction {
        return LoggingSequential(
            "GRAB_SPECIMEN",
            Loggable("GRAB_SPECIMEN", ParallelAction(InstantAction {
                outtakeActionWriter.write(StringMessage("GRAB_SPECIMEN"))
            }, grabber.setPosition(1))),
            lift.gotoDistance(3.0),
            Loggable("WAIT_FOR_ESCAPE", SleepAction(2.0)),
            lift.gotoDistance(PARAMS.liftPositions.topSpecimen)
        )
    }

    fun placeSpecimen(): LoggableAction {
        return LoggingSequential(
            "PLACE_SPECIMEN",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("PLACE_SPECIMEN"))
            }),
            lift.gotoDistance(PARAMS.liftPositions.topSpecimen - 2),
            Loggable("HOPE_LIFTS_CATCH_UP_AND_RELEASED", SleepAction(2.0)),
            Loggable("LET_GO", grabber.setPosition(0)),
            Loggable(
                "MOVE_ARM_IN", ParallelAction(
                    elbow.setPosition(0),
                    wrist.setPosition(0)
                )
            ),
            lift.gotoDistance(0.0)
        )
    }

    fun abortSpecimen(): LoggableAction {
        return LoggingSequential(
            "ABORT_SPECIMEN",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("ABORT_SPECIMEN"))
            }),
            lift.gotoDistance(3.0),
            Loggable(
                "MOVE_ARM_IN", ParallelAction(
                    elbow.setPosition(0),
                    wrist.setPosition(0)
                )
            ),
            lift.gotoDistance(0.0),
        )
    }

    override fun logState(uniqueName: String?) {
        lift.logState("$uniqueName [OUTTAKE_SLIDES]")
    }

    fun lockout() {
        lift.lockout()
    }

    fun unlock() {
        lift.unlock()
    }
}