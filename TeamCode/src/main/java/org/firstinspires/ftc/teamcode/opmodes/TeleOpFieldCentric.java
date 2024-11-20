package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.StaticLights;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpFieldCentric extends LinearOpMode {
    public static double SlowmodeSpeed = 0.4;
    public static double SlowmodeTurning = 0.5;
    public static double TriggerMin = 0.01;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        StaticLights.setup(hardwareMap, "blinkin");

        Button fieldMode = new Button();
        Button slowMode = new Button();

        Button specimenReady = new Button();

        Button progressSample = new Button();
        Button progressSpecimen = new Button();

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;
        if (Logging.DEBUG) {
            Action initAction = new ParallelAction(
                    intake.resetSlides(),
                    outtake.resetLifts()
            );
//        Run initialisation tasks
            p = new TelemetryPacket();
            while (!isStarted() && initAction.run(p)) {
                FtcDashboard.getInstance().sendTelemetryPacket(p);
                p = new TelemetryPacket();
                Logging.update();
                StaticLights.update();
            }
        }

        intake.lockout();
        outtake.lockout();

        PoseStorage.splitControls = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            visionDetection.update(driveBase.localizer, p);
            Logging.update();
            StaticLights.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            } else if (gamepad1.back) {
                PoseStorage.splitControls = false;
            }
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        LoggableAction sampleAction = null;
        LoggableAction finishingAction = null;
        boolean specimenGrabbed = false;
        SampleState sampleState = SampleState.Waiting;
        CapturingState captureState = CapturingState.None;
        FinishingState finishState = FinishingState.None;
        double lastTime = System.currentTimeMillis() / 1000.0;

        intake.unlock();
        outtake.unlock();

        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        while (opModeIsActive() && !isStopRequested()) {
            double secondsPassed = (System.currentTimeMillis() / 1000.0) - lastTime;
            lastTime = System.currentTimeMillis() / 1000.0;
            p = new TelemetryPacket();

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            } else if (gamepad1.back) {
                PoseStorage.splitControls = false;
            }

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;
            if (PoseStorage.shouldHallucinate) {
                StaticLights.specialState(
                        new RevBlinkinLedDriver.BlinkinPattern[]{
                                RevBlinkinLedDriver.BlinkinPattern.RED,
                                RevBlinkinLedDriver.BlinkinPattern.YELLOW
                        },
                        8
                );
            }

            if (PoseStorage.splitControls) {
                slowMode.update(gamepad1.left_bumper);
                fieldMode.update(gamepad1.x);

                if (gamepad2.dpad_right) {
                    PoseStorage.isRedAlliance = true;
                } else if (gamepad2.dpad_left) {
                    PoseStorage.isRedAlliance = false;
                }
            } else {
                slowMode.update(gamepad1.y);
                fieldMode.update(gamepad1.x);

                if (gamepad1.dpad_right) {
                    PoseStorage.isRedAlliance = true;
                } else if (gamepad1.dpad_left) {
                    PoseStorage.isRedAlliance = false;
                }
            }

            driveBase.update(p);

            Pose2d poseEstimate = driveBase.localizer.currentPose;
            input = input.times(slowMode.val ? SlowmodeSpeed : 1);

            if (fieldMode.val) {
                input = poseEstimate.heading.inverse().plus((PoseStorage.isRedAlliance ? 1 : -1) * Math.PI / 2).times(input);
            }

            PoseVelocity2d drivePower = new PoseVelocity2d(
                    input,
                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
            );

            Logging.DEBUG("X Input", input.x);
            Logging.DEBUG("Y Input", input.y);

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            driveBase.setDrivePowers(drivePower);

            if (sampleState == SampleState.SpecimenIntake && (sampleAction == null || sampleAction.getName().equals("CANCELABLE_ADJUST"))) {
                double change = secondsPassed * 2 * (PoseStorage.splitControls ? gamepad2.right_trigger - gamepad2.left_trigger : gamepad1.right_trigger - gamepad1.left_trigger);
                Outtake.PARAMS.getLiftPositions().topSpecimen += change;
                Logging.LOG("NEW_POSITION", Outtake.PARAMS.getLiftPositions().topSpecimen);
                sampleAction = new Loggable("CANCELABLE_ADJUST", outtake.getLift().gotoDistance(Outtake.PARAMS.getLiftPositions().topSpecimen));
            }

            Logging.LOG("SAMPLE_STATE", sampleState);
            if (sampleAction != null) {
                Logging.DEBUG("SAMPLE_ACTION", sampleAction.getName());

                if (!sampleAction.run(p)) {
                    Logging.LOG("SAMPLE_ACTION_FINISHED");
                    sampleAction = null;

                    switch (sampleState) {
                        case Outtaking:
                            finishState = FinishingState.Outtake;
                            finishingAction = outtake.retractArm();
                            break;

                        case Waiting:
                            StaticLights.getColors()[0] = StaticLights.getColors()[1];
                            sampleState = SampleState.Captured;
                            sampleAction = new LoggingSequential("TRANSFER",
                                    intake.retractSlides(),
                                    intake.transfer(),
                                    outtake.pickupInternalSample()
                            );
                            break;

                        case SpecimenIntake:
                            sampleState = SampleState.Waiting;
                            finishState = FinishingState.Outtake;
                            finishingAction = outtake.returnSpecimen();
                            break;
                    }
                }
            }

            if (finishingAction != null && !finishingAction.run(p)) {
                finishState = FinishingState.None;
                finishingAction = null;
            }

            if (specimenReady.update((PoseStorage.splitControls ? gamepad2 : gamepad1).b)
                    && sampleAction == null
                    && (sampleState == SampleState.Waiting || sampleState == SampleState.Captured || sampleState == SampleState.SpecimenWait)) {
                if (finishState == FinishingState.Outtake) {
                    finishState = FinishingState.None;
                    finishingAction = null;
                }

                if (sampleState == SampleState.SpecimenWait) {
                    // Cancel specimen
                    sampleState = specimenGrabbed ? SampleState.Captured : SampleState.Waiting; // It's probably actually a sample if we cancel here
                    finishingAction = outtake.abortSpecimen();
                    finishState = FinishingState.Outtake;
                }

                specimenGrabbed = sampleState == SampleState.Captured;
                sampleAction = outtake.specimenReady(!specimenGrabbed);
                sampleState = SampleState.SpecimenWait;
            }

            if (sampleState == SampleState.SpecimenWait && progressSpecimen.update((PoseStorage.splitControls ? gamepad2 : gamepad1).left_bumper)) {
                specimenGrabbed = !specimenGrabbed;
                sampleAction = outtake.grabber(!specimenGrabbed);
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).a && sampleState == SampleState.Waiting) {
                if (finishState == FinishingState.Intake) {
                    finishState = FinishingState.None;
                    finishingAction = null;
                }

                if (sampleAction == null) {
                    Logging.LOG("Running close capture sequence");
                    sampleAction = intake.captureSample(true);
                    captureState = CapturingState.Close;
                }
            } else if (captureState == CapturingState.Close) {
                finishState = FinishingState.Intake;
                finishingAction = intake.retractSlides();
            }

            float specificTrigger = (PoseStorage.splitControls ? gamepad2 : gamepad1).left_trigger;
            float sharedTrigger = (PoseStorage.splitControls ? gamepad2 : gamepad1).right_trigger;
            if (specificTrigger > TriggerMin || sharedTrigger > TriggerMin && sampleState == SampleState.Waiting) {
                if (finishState == FinishingState.Intake) {
                    finishState = FinishingState.None;
                    finishingAction = null;
                }

                if (sampleAction == null) {
                    Logging.LOG("Running far capture sequence");
                    boolean shared = sharedTrigger > TriggerMin;
                    sampleAction = intake.captureSample(
                            shared,
                            () -> Intake.minExtension + ((shared ? sharedTrigger : specificTrigger) * (Intake.maxExtension - Intake.minExtension)));
                    captureState = CapturingState.Far;
                }
            } else if (captureState == CapturingState.Far) {
                sampleAction = null;
                captureState = CapturingState.None;
                finishState = FinishingState.Intake;
                finishingAction = intake.retractSlides();
            }

            if (progressSample.update((PoseStorage.splitControls ? gamepad2 : gamepad1).right_bumper) && sampleAction == null) {
                switch (sampleState) {
                    case Captured:
                        sampleAction = outtake.topBasket();
                        sampleState = SampleState.Outtaking;
                        break;
                    case Outtaking:
                        sampleAction = outtake.dropSample();
                        sampleState = SampleState.Clearing;
                        break;
                    case Clearing:
                        finishingAction = outtake.homePosition();
                        finishState = FinishingState.Outtake;
                        sampleState = SampleState.Waiting;
                        break;
                    case SpecimenWait:
                        if (specimenGrabbed) {
                            sampleAction = new LoggingSequential(
                                    "DEPLOY_SPECIMEN",
                                    outtake.raiseSpecimen(),
                                    outtake.ensureSpecimenPlaced()
                            );
                            sampleState = SampleState.SpecimenIntake;
                        } else {
                            finishingAction = outtake.abortSpecimen();
                            finishState = FinishingState.Outtake;
                        }
                        break;
                }
            }

            if (matchTimer.timeRemaining(TimeUnit.SECONDS) < 30 && matchTimer.timeRemaining(TimeUnit.SECONDS) > 28) {
                StaticLights.specialState(
                        new RevBlinkinLedDriver.BlinkinPattern[]{
                                RevBlinkinLedDriver.BlinkinPattern.HOT_PINK,
                                RevBlinkinLedDriver.BlinkinPattern.WHITE
                        },
                        8
                );
            }
            if (matchTimer.timeRemaining(TimeUnit.SECONDS) < 2) {
                StaticLights.specialState(
                        new RevBlinkinLedDriver.BlinkinPattern[]{RevBlinkinLedDriver.BlinkinPattern.GREEN},
                        1
                );
            }

            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");
            intake.logState("[TELEOP]");
            outtake.logState("[TELEOP]");
            Logging.update();
            StaticLights.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    enum SampleState {
        Waiting,
        Captured,
        Outtaking,
        Clearing,

        SpecimenWait,
        SpecimenIntake,
    }

    enum CapturingState {
        Close,
        Far,
        None
    }

    enum FinishingState {
        Intake,
        Outtake,
        None
    }
}
