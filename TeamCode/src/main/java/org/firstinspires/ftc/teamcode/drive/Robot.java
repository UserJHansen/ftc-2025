package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.LiftArmAssembly;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.VisionBase;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras.BackwardCamera;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public class Robot {
    public final SampleMecanumDrive driveBase;
    public final LiftArmAssembly liftArmAssembly;

    public boolean adjustedPose = false;

    public Robot(HardwareMap hardwareMap, Boolean auto) {
        driveBase = new SampleMecanumDrive(hardwareMap);
        liftArmAssembly = new LiftArmAssembly(hardwareMap);

        driveBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveBase.setPoseEstimate(PoseStorage.currentPose);
    }

    //    Called on a loop during the init stage, before the start button is pressed
    public void updateInit() {
        updateBase();
    }

    public void updateBase() {
        Pose2d poseEstimate = driveBase.getPoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        driveBase.update(packet);
        liftArmAssembly.update();

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

        Logging.update();

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    //    Called on a loop during the opmode stage, after the start button is pressed
    public void update() {
        updateBase();
    }
}
