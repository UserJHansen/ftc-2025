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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Logging;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.doWhile;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;

import java.util.List;

@TeleOp(group = "advanced", name = "Teleop")
@Config
public class TeleOpFieldCentric extends LinearOpMode {
    public static double SlowmodeSpeed = 0.8;
    public static double SlowmodeTurning = 0.8;

    enum SampleState {
        Waiting,
        Intaking,
        Outtaking,

        SpecimenWait,
        SpecimenIntake,
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        Button fieldMode = new Button();
        Button slowMode = new Button();
        Button inverted = new Button();

        Button specimenReady = new Button();

        Button progressSample = new Button();
        Button indicateFailure = new Button();

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Action initAction = new ParallelAction(
                intake.resetSlides(),
                outtake.resetLifts()
        );

//        Run initialisation tasks
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
        }

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Action driveAction = null;
        Action sampleAction = null;
        SampleState sampleState = SampleState.Waiting;

        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();
            Pose2d poseEstimate = driveBase.localizer.currentPose;
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).times(slowMode.val ? SlowmodeSpeed : 1);

            if (fieldMode.val) {
                input = poseEstimate.heading.inverse().times(input);
            }

            if (inverted.val) {
                input = Rotation2d.fromDouble(Math.PI).times(input);
            }

            PoseVelocity2d drivePower = new PoseVelocity2d(
                    input,
                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
            );

            Logging.DEBUG("X Input", input.x);
            Logging.DEBUG("Y Input", input.y);

            slowMode.update(gamepad1.b);
            fieldMode.update(gamepad1.x);
            inverted.update(gamepad1.y);

            visionDetection.update(driveBase.localizer, p);

            if (driveAction != null && !driveAction.run(p)) {
                driveAction = null;
            };
            if (sampleAction != null && !sampleAction.run(p)) {
                sampleAction = null;
            };

            if (specimenReady.update(gamepad1.a) && sampleAction == null) {
                sampleAction = outtake.specimenReady();
                sampleState = SampleState.SpecimenWait;
            }

            if (progressSample.update(gamepad1.right_bumper) && sampleAction == null) {
                switch (sampleState) {
                    case Waiting:
                        sampleAction = new SequentialAction(
                                intake.captureSample(),
                                outtake.readyForTransfer(),
                                new doWhile(
                                    intake.transfer(),
                                    outtake.waitForTransfer()
                                ),
                                intake.stopTransfer(),
                                outtake.pickupInternalSample()
                        );
                        sampleState = SampleState.Intaking;
                        break;
                    case Intaking:
                        sampleAction = outtake.topBasket();
                        sampleState = SampleState.Outtaking;
                        break;
                    case Outtaking:
                        sampleAction = outtake.dropSample();
                        sampleState = SampleState.Waiting;
                        break;

                    case SpecimenWait:
                        sampleAction = outtake.grabSpecimen();
                        sampleState = SampleState.SpecimenIntake;
                        break;

                    case SpecimenIntake:
                        sampleAction = outtake.placeSpecimen();
                        sampleState = SampleState.Waiting;
                }
            }

            if (indicateFailure.update(gamepad1.left_bumper)) {
                switch (sampleState) {
                    case Waiting: // Probably a false grab?
                        if (sampleAction != null) {
                            sampleAction = new SequentialAction(
                                    intake.captureSample(),
                                    outtake.readyForTransfer(),
                                    new doWhile(
                                            intake.transfer(),
                                            outtake.waitForTransfer()
                                    ),
                                    intake.stopTransfer(),
                                    outtake.pickupInternalSample()
                            );
                        }
                        break;
                    case SpecimenWait:
                        sampleAction = outtake.abortSpecimen();
                        break;
                    case SpecimenIntake: // False grab also probably
                        sampleAction = outtake.grabSpecimen();
                        break;
                }
            }

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            driveBase.setDrivePowers(drivePower);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}