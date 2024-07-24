package org.firstinspires.ftc.teamcode.drive.advanced;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.ArmTarget;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

import java.util.Arrays;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced", name = "0 Teleop")
@Config
public class TeleOpFieldCentric extends LinearOpMode {
    public static double SlowmodeSpeed = 0.8;
    public static double SlowmodeTurning = 0.8;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, false);
        Button fieldMode = new Button(false);
        Button slowMode = new Button(false);
        Button inverted = new Button(false);
        Button forwardState = new Button(false, (none) -> {
            robot.liftArmAssembly.target = robot.liftArmAssembly.target.next;
        });
        Button revertState = new Button(false, (none) -> {
            robot.liftArmAssembly.reset();
        });

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.driveBase.setPoseEstimate(PoseStorage.currentPose);

        while (!isStarted()) {
            robot.updateInit();

            sleep(10);
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = robot.driveBase.getPoseEstimate();
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).times(slowMode.val ? SlowmodeSpeed : 1);

            if (fieldMode.val) {
                input = input.rotated(-poseEstimate.getHeading());
            }

            if (inverted.val) {
                input = input.rotated(Math.PI);
            }

            Pose2d drivePower = new Pose2d(
                    input.getX(),
                    input.getY(),
                    -gamepad1.right_stick_x * (slowMode.val ? SlowmodeTurning : 1)
            );

            robot.update();
            slowMode.update(gamepad1.b);
            fieldMode.update(gamepad1.x);
            inverted.update(gamepad1.y);
            forwardState.update(gamepad2.right_bumper);
            revertState.update(gamepad2.left_bumper);

            int intake = gamepad2.b || robot.liftArmAssembly.target == ArmTarget.Primed
                    ? 2 :
                    robot.liftArmAssembly.target == ArmTarget.Waiting || robot.liftArmAssembly.target == ArmTarget.Caught
                            ? 1 : 0;
            robot.intake.changeTo(intake);

            robot.liftArmAssembly.update(gamepad2.right_trigger, gamepad2.left_trigger);
            if (gamepad2.back && gamepad2.dpad_down) {
                robot.liftArmAssembly.endgame(!gamepad2.dpad_left);
            }

            robot.plane.update(gamepad2.x);

//            LIGHT STATE HANDLING
            boolean hasLock = robot.adjustedPose;
            RevBlinkinLedDriver.BlinkinPattern firstBlink = slowMode.val ? RevBlinkinLedDriver.BlinkinPattern.AQUA : fieldMode.val ? RevBlinkinLedDriver.BlinkinPattern.VIOLET : hasLock ? RevBlinkinLedDriver.BlinkinPattern.GREEN : RevBlinkinLedDriver.BlinkinPattern.RED;

            boolean pixelL = robot.leftPixelTrigger.val;
            boolean pixelR = robot.rightPixelTrigger.val;
            RevBlinkinLedDriver.BlinkinPattern secondBlink;
            if (pixelL && pixelR) {
                secondBlink = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (pixelL) {
                secondBlink = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            } else if (pixelR) {
                secondBlink = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            } else {
                secondBlink = RevBlinkinLedDriver.BlinkinPattern.RED;
            }

            robot.statusLEDs.changeTo(Arrays.asList(firstBlink, secondBlink));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            robot.driveBase.setWeightedDrivePower(drivePower);
        }
    }
}