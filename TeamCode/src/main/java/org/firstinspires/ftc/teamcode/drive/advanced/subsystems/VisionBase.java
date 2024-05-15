package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class VisionBase {
    public static AprilTagLibrary getFixedTags() {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(60.25f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(60.25f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(60.25f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(60.25f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(60.25f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    @SuppressLint("DefaultLocale")
    public static void logTagPosition(Canvas fieldOverlay, Pose2d poseEstimate, AprilTagDetection tag, Camera camera) {
        if (tag.metadata == null) {
            Logging.DEBUG(String.format("\n==== (ID %d) Unknown", tag.id));
            Logging.DEBUG(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
            return;
        }


//                Draw raw position relative to 0, 0
        if (Logging.DEBUG) {
            fieldOverlay.setStroke("#800080");
            DashboardUtil.drawRobot(fieldOverlay, new Pose2d(tag.ftcPose.y, -tag.ftcPose.x, Math.toRadians(tag.ftcPose.yaw) - Math.PI));
        }

        fieldOverlay.setStroke("#00ffff");
        DashboardUtil.drawRobot(fieldOverlay, VisionMath.getObjectPose(poseEstimate, tag, camera.getCameraPos()));

        if (tag.id <= 10) {
            fieldOverlay.setStroke("#777777");
            DashboardUtil.drawRobot(fieldOverlay, VisionMath.getRobotPose(tag, camera.getCameraPos()));
        }


        Logging.LOG(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
        Logging.LOG("Field Position", VisionMath.getObjectPose(poseEstimate, tag, camera.getCameraPos()).toString());
        Logging.LOG(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
        Logging.DEBUG(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
        Logging.DEBUG(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
    }
}
