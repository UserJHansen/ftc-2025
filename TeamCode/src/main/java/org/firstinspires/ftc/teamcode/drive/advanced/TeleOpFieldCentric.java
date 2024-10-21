package org.firstinspires.ftc.teamcode.drive.advanced;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.ArmTarget;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

import java.util.List;

@TeleOp(group = "advanced", name = "Teleop")
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
        Button forwardState = new Button(false, (none) -> robot.liftArmAssembly.target = robot.liftArmAssembly.target.next);
        Button revertState = new Button(false, (none) -> robot.liftArmAssembly.reset());

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.driveBase.setPoseEstimate(PoseStorage.currentPose);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (!isStarted()) {
            robot.updateInit();

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }

        if (isStopRequested()) return;

        robot.liftArmAssembly.resetLifts();

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

            Logging.DEBUG("X Input", input.getX());
            Logging.DEBUG("Y Input", input.getY());

            robot.update();

            if (gamepad1.a) {
                robot.liftArmAssembly.target = ArmTarget.SpecimenSmallLift;
            }

            slowMode.update(gamepad1.b);
            fieldMode.update(gamepad1.x);
            inverted.update(gamepad1.y);
            forwardState.update(gamepad1.right_bumper);
            revertState.update(gamepad1.left_bumper);

            if (gamepad1.back) { // SOMETHING VERY VERY BAD HAS HAPPENED, GET THE SAMPLE OUT
                robot.liftArmAssembly.intakeMechanism.overrideMotor(1); // Run the motor to throw the current sample out if its trapped under the robot
            }

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            robot.driveBase.setWeightedDrivePower(drivePower);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }
}