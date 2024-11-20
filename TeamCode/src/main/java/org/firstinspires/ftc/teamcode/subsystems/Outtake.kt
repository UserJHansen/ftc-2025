package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.CurrentCutoff
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState
import org.firstinspires.ftc.teamcode.messages.StringMessage

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField
        var P_Outake = 10.0

        @JvmField
        var SpecimenCurrentTrigger = 8.0

        class LiftPositions {
            @JvmField
            var topBasket = 27.0

            @JvmField
            var topSpecimen = 12.5
        }

        class GrabberLimits {
            @JvmField
            var waiting = 1.0

            @JvmField
            var grabbing = 0.0

            @JvmField
            var loose = 0.2
        }

        class ElbowLimits {
            @JvmField
            var intake = 0.49

            @JvmField
            var deposit = 1.0

            @JvmField
            var specimen = 0.57

            @JvmField
            var specimenOtherSide = 0.235
        }

        class WristLimits {
            @JvmField
            var intake = 0.78

            @JvmField
            var specimen = 0.48

            @JvmField
            var deposit = 0.4

            @JvmField
            var specimenOtherSide = 0.84

            @JvmField
            var specimenSecure = 0.9
        }

        val liftPositions = LiftPositions()
        val grabberLimits = GrabberLimits()
        val elbowLimits = ElbowLimits()
        val wristLimits = WristLimits()
    }

    val lift = Lift(hardwareMap, "outtakeSlides", DcMotorSimple.Direction.FORWARD, PARAMS.P_Outake)
    val endLimit = DigitalInput(hardwareMap, "outtakeLimit")

    val grabber = ServoMultiState(
        hardwareMap, "outtakeGrabber",
        doubleArrayOf(
            PARAMS.grabberLimits.waiting,
            PARAMS.grabberLimits.grabbing,
            PARAMS.grabberLimits.loose
        ), 0.4
    )
    val elbow = ServoMultiState(
        hardwareMap, "elbow",
        doubleArrayOf(
            PARAMS.elbowLimits.intake,
            PARAMS.elbowLimits.deposit,
            PARAMS.elbowLimits.specimen,
            PARAMS.elbowLimits.specimenOtherSide,
        ), 0.578
    )
    val wrist = ServoMultiState(
        hardwareMap, "wrist",
        doubleArrayOf(
            PARAMS.wristLimits.intake,
            PARAMS.wristLimits.specimen,
            PARAMS.wristLimits.deposit,
            PARAMS.wristLimits.specimenOtherSide,
            PARAMS.wristLimits.specimenSecure
        ), 1.17
    )

    private val outtakeActionWriter = DownsampledWriter("OUTTAKE_ACTION", 50_000_000)

    fun resetLifts(): LoggableAction {
        outtakeActionWriter.write(StringMessage("RESET_LIFTS"))
        return lift.resetPosition(endLimit.until(false))
    }

    fun homePosition(): LoggableAction {
        return Loggable(
            "MOVE_ARM_TRANSFER", SequentialAction(
                InstantAction {
                    outtakeActionWriter.write(StringMessage("MOVE_ARM_TRANSFER"))
                },
                grabber.setPosition(0),
                wrist.setPosition(0),
                elbow.setPosition(0),
                SleepAction(0.3)
            )
        )
    }

    fun pickupInternalSample(): LoggableAction {
        return LoggingSequential(
            "PICKUP_INTERNAL_SAMPLE",
            Loggable("GRAB_SAMPLE", grabber.setPosition(1)),
            Loggable("MOVE_OUT", elbow.setPosition(2)),
        )
    }

    fun topBasket(): LoggableAction {
        return LoggingSequential(
            "GOTO_TOP_BASKET",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("TOP_BASKET"))
            }),
            lift.goToThroughWhile(
                liftPositions.topBasket,
                liftPositions.topBasket / 2,
                Loggable(
                    "EXTEND_ARM",
                    ParallelAction(
                        elbow.setPosition(1),
                        wrist.setPosition(2),
                    )
                )
            ),
        )
    }

    fun dropSample(): LoggableAction {
        return Loggable("RELEASE_SAMPLE", ParallelAction(InstantAction {
            outtakeActionWriter.write(StringMessage("DROP_SAMPLE"))
        }, grabber.setPosition(0)))
    }

    fun retractArm(): LoggableAction = LoggingSequential(
        "RETRACT_ARM",
        Loggable("LOG", InstantAction {
            outtakeActionWriter.write(StringMessage("RETRACT"))
        }),

        Loggable(
            "MOVE_ARM_IN", ParallelAction(
                grabber.setPosition(0),
                elbow.setPosition(0),
                wrist.setPosition(0)
            )
        ),
        lift.gotoDistance(0.0)
    )

    fun specimenReady(open: Boolean): LoggableAction {
        return LoggingSequential(
            "SPECIMEN_READY",
            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.WHITE,
                    RevBlinkinLedDriver.BlinkinPattern.GREEN
                )
            ),
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("SPECIMEN_READY"))
            }),
            lift.gotoDistance(4.0),
            Loggable(
                "MOVE_ARM_OUT", ParallelAction(
                    grabber.setPosition(1),
                    elbow.setPosition(2),
                    wrist.setPosition(1)
                )
            ),
            lift.gotoDistance(2.0),
            Loggable("MOVE_GRABBER_TO_POSITION", grabber.setPosition(if (open) 0 else 1))
        )
    }

    fun grabber(open: Boolean): LoggableAction {
        return LoggingSequential(
            "MOVE_GRABBER",
            Loggable("MOVE_GRABBER", ParallelAction(InstantAction {
                outtakeActionWriter.write(StringMessage("MOVE_GRABBER"))
            }, grabber.setPosition(if (open) 0 else 1))),
        )
    }

    fun raiseSpecimen(): LoggableAction {
        return LoggingSequential(
            "GRAB_SPECIMEN",
            Loggable("GRAB_SPECIMEN", InstantAction {
                outtakeActionWriter.write(StringMessage("GRAB_SPECIMEN"))
            }),
            lift.goToThroughWhile(
                liftPositions.topSpecimen,
                liftPositions.topSpecimen / 2,
                LoggingSequential(
                    "MOVE_ARM_OUT",

                    Loggable(
                        "MOVE_ARM_OUT",
                        ParallelAction(elbow.setPosition(3), wrist.setPosition(3))
                    ),
                    Loggable(
                        "GRAB_DROP", ParallelAction(
                            grabber.setPosition(2),
                            SleepAction(0.2),
                        )
                    ),
                    Loggable("GRAB", grabber.setPosition(1)),
                )
            )
        )
    }

    fun placeSpecimen(): LoggableAction {
        return LoggingSequential(
            "PLACE_SPECIMEN",
            lift.gotoDistance(liftPositions.topSpecimen),
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("PLACE_SPECIMEN"))
                lift.liftMotor.setPositionPIDFCoefficients(25.0)
            }),
            Loggable(
                "GRAB", ParallelAction(
                    grabber.setPosition(1),
                    wrist.setPosition(4)
                )
            ),
            lift.gotoDistance(PARAMS.liftPositions.topSpecimen + 6, 0.25),
            Loggable("WAIT_FOR_FINAL", SleepAction(0.5)),
            Loggable(
                "PID_NORMAL",
                InstantAction { lift.liftMotor.setPositionPIDFCoefficients(PARAMS.P_Outake) })
        )
    }

    fun ensureSpecimenPlaced(): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = if (currentAction != null) currentAction!!.name else "SPECIMEN_PLACE"
            var currentAction: LoggableAction? = null;
            val currentTriggerAction = SequentialAction(
                CurrentCutoff(lift.liftMotor).above(SpecimenCurrentTrigger),
                CurrentCutoff(lift.liftMotor).below(SpecimenCurrentTrigger)
            )

            override fun run(p: TelemetryPacket): Boolean {
                if (currentAction == null || !currentAction!!.run(p)) {
                    currentAction = placeSpecimen()
                }

                return currentTriggerAction.run(p)
            }

        }
    }

    fun returnSpecimen(): LoggableAction {
        return LoggingSequential(
            "RETURN_SPECIMEN",
            Loggable("LET_GO", grabber.setPosition(0)),
            Loggable(
                "MOVE_SAFE", ParallelAction(
                    elbow.setPosition(2),
                    wrist.setPosition(0)
                )
            ),
            lift.gotoDistance(0.0),
            Loggable(
                "MOVE_ARM_IN", ParallelAction(
                    elbow.setPosition(0),
                )
            ),
            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.BLACK
                )
            ),
        )
    }

    fun abortSpecimen(): LoggableAction {
        return LoggingSequential(
            "ABORT_SPECIMEN",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("ABORT_SPECIMEN"))
            }),
            lift.gotoDistance(4.0),
            Loggable(
                "MOVE_SAFE", ParallelAction(
                    elbow.setPosition(2),
                    wrist.setPosition(0)
                )
            ),
            lift.gotoDistance(0.0),
            Loggable(
                "MOVE_ARM_IN", ParallelAction(
                    elbow.setPosition(0),
                )
            ),
            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.BLACK
                )
            )
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
