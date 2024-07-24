package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.CameraPoseAdjust;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.DistanceTrigger;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.BackwardCamera;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.ForwardCamera;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.LEDStrip;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.LiftArmAssembly;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.MotorMultiState;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.ServoToggle;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.SwappableCameras;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.VisionBase;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public class Robot {
    public final LiftArmAssembly liftArmAssembly;
    public final MotorMultiState intake;
    public final DistanceTrigger leftPixelTrigger;
    public final DistanceTrigger rightPixelTrigger;
    public boolean adjustedPose = false;
    public final LEDStrip statusLEDs;
    public final SampleMecanumDrive driveBase;
    public final ServoToggle plane;

    public static double intakeSpeed = -1;

    public Robot(HardwareMap hardwareMap, Boolean auto) {
        driveBase = new SampleMecanumDrive(hardwareMap);
        liftArmAssembly = new LiftArmAssembly(hardwareMap);

        leftPixelTrigger = new DistanceTrigger(5.5, hardwareMap.get(ColorRangeSensor.class, "leftPixel"));
        rightPixelTrigger = new DistanceTrigger(5.5, hardwareMap.get(ColorRangeSensor.class, "rightPixel"));

        intake = new MotorMultiState(hardwareMap, "intake", new double[]{0.0, -intakeSpeed, intakeSpeed});

        if (!auto) {
            plane = new ServoToggle(hardwareMap, "plane", 1.0, 0.0);
        } else {
            plane = null;
        }

        statusLEDs = new LEDStrip(hardwareMap, "statusLEDs");

        driveBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveBase.setPoseEstimate(PoseStorage.currentPose);

        statusLEDs.changeTo(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    //    Called on a loop during the init stage, before the start button is pressed
    public void updateInit() {
        updateBase();
        statusLEDs.update();
    }

    public void updateBase() {
        Pose2d poseEstimate = driveBase.getPoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        driveBase.update(packet);


        AprilTagMetadata[] fieldTags = VisionBase.getFixedTags().getAllTags();
        fieldOverlay.setFill("#e000e0");
        for (AprilTagMetadata tag : fieldTags) {
            Vector2d[] points = VisionMath.getAprilTagPoints(tag);
            double[] xPoints = new double[4];
            double[] yPoints = new double[4];

            for (int i = 0; i < 4; i++) {
                xPoints[i] = points[i].getX();
                yPoints[i] = points[i].getY();
            }

            fieldOverlay.fillPolygon(xPoints, yPoints);
        }

        Logging.LOG("# AprilTags Detected", VisionDetection.detections.size());
        packet.put("ForwardTags", VisionDetection.detections.size());
        for (AprilTagDetection tag : VisionDetection.detections) {
            VisionBase.logTagPosition(fieldOverlay, poseEstimate, tag, new BackwardCamera());
        }

        // Print pose to telemetry
        Logging.LOG("x", poseEstimate.getX());
        Logging.LOG("y", poseEstimate.getY());
        Logging.LOG("heading", poseEstimate.getHeading());

        Logging.LOG("leftPixel", leftPixelTrigger.val);
        Logging.LOG("rightPixel", rightPixelTrigger.val);
        Logging.LOG("leftPixelD", leftPixelTrigger.sensor.getDistance(DistanceUnit.CM));
        Logging.LOG("rightPixelD", rightPixelTrigger.sensor.getDistance(DistanceUnit.CM));

        Logging.LOG(String.valueOf(liftArmAssembly.target));

        Logging.LOG("intake", intake.get());

        Logging.update();

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    //    Called on a loop during the opmode stage, after the start button is pressed
    public void update() {
        updateBase();
        leftPixelTrigger.update();
        rightPixelTrigger.update();
        liftArmAssembly.update(leftPixelTrigger.val, rightPixelTrigger.val);
        statusLEDs.update();
    }
}
