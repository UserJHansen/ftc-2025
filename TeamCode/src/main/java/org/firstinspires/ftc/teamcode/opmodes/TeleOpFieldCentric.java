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
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import java.util.List;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpFieldCentric extends LinearOpMode {
    public static double SlowmodeSpeed = 0.4;
    public static double SlowmodeTurning = 0.5;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        Button fieldMode = new Button();
        Button slowMode = new Button();

        Button specimenReady = new Button();

        Button progressSample = new Button();
        Button indicateFailure = new Button();

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
            FtcDashboard.getInstance().sendTelemetryPacket(p);

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            }
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        LoggableAction sampleAction = null;
        SampleState sampleState = SampleState.Waiting;
        double lastTime = System.currentTimeMillis() / 1000.0;

        intake.unlock();
        outtake.unlock();

        PoseStorage.isInit = false;
        while (opModeIsActive() && !isStopRequested()) {
            double secondsPassed = (System.currentTimeMillis() / 1000.0) - lastTime;
            lastTime = System.currentTimeMillis() / 1000.0;
            p = new TelemetryPacket();

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
                fieldMode.val = true;
            }

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;
            if (PoseStorage.splitControls) {
                slowMode.update(gamepad1.left_bumper);
                fieldMode.update(gamepad1.x);

                if (gamepad2.b) {
                    PoseStorage.isRedAlliance = true;
                } else if (gamepad2.x) {
                    PoseStorage.isRedAlliance = false;
                }
            } else {
                slowMode.update(gamepad1.b);
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
            } else if (!PoseStorage.splitControls && (sampleState == SampleState.Outtaking || sampleState == SampleState.SpecimenWait)) {
                input = Rotation2d.fromDouble(Math.PI).times(input);
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
                Outtake.Companion.getPARAMS().getLiftPositions().topSpecimen += change;
                Logging.LOG("NEW_POSITION", Outtake.Companion.getPARAMS().getLiftPositions().topSpecimen);
                sampleAction = new Loggable("CANCELABLE_ADJUST", outtake.getLift().gotoDistance(Outtake.Companion.getPARAMS().getLiftPositions().topSpecimen));
            }

            Logging.LOG("SAMPLE_STATE", sampleState);
            if (sampleAction != null) {
                Logging.DEBUG("SAMPLE_ACTION", sampleAction.getName());

                if (!sampleAction.run(p)) {
                    Logging.LOG("SAMPLE_ACTION_FINISHED");
                    sampleAction = null;
                }
            }

            if (specimenReady.update((PoseStorage.splitControls ? gamepad2 : gamepad1).a) && sampleAction == null && sampleState == SampleState.Waiting) {
                sampleAction = outtake.specimenReady();
                sampleState = SampleState.SpecimenWait;
            }

            if (sampleState == SampleState.Waiting && sampleAction == null && (PoseStorage.splitControls ? gamepad2 : gamepad1).dpad_down) {
                Logging.LOG("Running close capture sequence");
                sampleAction = new Loggable(
                        "INTAKE",
                        new ParallelAction(
                                outtake.readyForIntake(),
                                intake.captureSample(true, true)
                        )
                );
                sampleState = SampleState.Intaking;
            }

            if (progressSample.update((PoseStorage.splitControls ? gamepad2 : gamepad1).right_bumper) && sampleAction == null) {
                switch (sampleState) {
                    case Waiting:
                        Logging.LOG("Running capture sequence");
                        sampleAction = new LoggingSequential("INTAKE_SEQUENCE",
                                new Loggable("INTAKE", new ParallelAction(
                                        outtake.readyForIntake(),
                                        intake.captureSample(true)
                                )),
                                outtake.pickupInternalSample()
                        );
                        sampleState = SampleState.Intaking;
                        break;
                    case Intaking:
                        Logging.LOG("Running deploy sequence");
                        sampleAction = outtake.topBasket();
                        sampleState = SampleState.Outtaking;
                        break;
                    case Outtaking:
                        Logging.LOG("Running drop sequence");
                        sampleAction = outtake.dropSample();
                        sampleState = SampleState.ReturnArm;
                        break;
                    case ReturnArm:
                        sampleAction = outtake.retractArm();
                        sampleState = SampleState.Waiting;
                        break;

                    case SpecimenWait:
                        Logging.LOG("Running specimen grab sequence");
                        sampleAction = outtake.grabSpecimen();
                        sampleState = SampleState.SpecimenIntake;
                        break;

                    case SpecimenIntake:
                        sampleAction = outtake.placeSpecimen();
                        sampleState = SampleState.Waiting;
                        break;
                }
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).start && (sampleState == SampleState.Intaking)) {
                sampleState = SampleState.Waiting;
                sampleAction = intake.retractSlides();
            }

            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);

            if (indicateFailure.update((PoseStorage.splitControls ? gamepad2 : gamepad1).left_bumper)) {
                switch (sampleState) {
                    case Intaking: // Probably a false grab?
                        sampleAction = new LoggingSequential(
                                "INTAKE",
                                outtake.readyForIntake(),
                                intake.captureSample(false),
                                outtake.pickupInternalSample()
                        );
                        break;
                    case SpecimenWait:
                        sampleAction = outtake.abortSpecimen();
                        sampleState = SampleState.Waiting;
                        break;
                    case SpecimenIntake: // False grab also probably
                        sampleAction = outtake.retryGrabSpecimen();
                        sampleState = SampleState.SpecimenWait;
                        break;
                }
            }

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");
            intake.logState("[TELEOP]");
            outtake.logState("[TELEOP]");
            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    enum SampleState {
        Waiting,
        Intaking,
        Outtaking,
        ReturnArm,

        SpecimenWait,
        SpecimenIntake,
    }
}