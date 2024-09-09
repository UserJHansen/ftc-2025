package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras.Camera;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class VisionBase {
    public static AprilTagLibrary getFixedTags() {
        return AprilTagGameDatabase.getCurrentGameTagLibrary();
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
