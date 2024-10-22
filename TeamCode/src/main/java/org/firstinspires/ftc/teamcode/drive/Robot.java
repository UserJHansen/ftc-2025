package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.LiftArmAssembly;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public class Robot {
    public final MecanumDrive driveBase;
    public final LiftArmAssembly liftArmAssembly;
    public final VisionDetection visionDetection;

    public boolean adjustedPose = false;

    public Robot(HardwareMap hardwareMap, Boolean auto) {
        driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        liftArmAssembly = new LiftArmAssembly(hardwareMap);
        visionDetection = new VisionDetection(hardwareMap);
    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        driveBase.update(packet);
        visionDetection.update(driveBase.localizer, packet);

        AprilTagMetadata[] fieldTags = AprilTagGameDatabase.getCurrentGameTagLibrary().getAllTags();
        fieldOverlay.setFill("#e000e0");
        for (AprilTagMetadata tag : fieldTags) {
            Vector2d[] points = VisionMath.getAprilTagPoints(tag);
            double[] xPoints = new double[4];
            double[] yPoints = new double[4];

            for (int i = 0; i < 4; i++) {
                xPoints[i] = points[i].x;
                yPoints[i] = points[i].y;
            }

            fieldOverlay.fillPolygon(xPoints, yPoints);
        }

        Logging.update();

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
