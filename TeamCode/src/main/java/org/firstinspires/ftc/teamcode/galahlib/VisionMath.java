package org.firstinspires.ftc.teamcode.galahlib;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class VisionMath {

    public static Vector2d[] getAprilTagPoints(AprilTagMetadata tag) {
        Vector2d pos = new Vector2d(tag.fieldPosition.get(0), tag.fieldPosition.get(1));
        double width = tag.tagsize;

        double t3 = 2.0 * (tag.fieldOrientation.w * tag.fieldOrientation.z + tag.fieldOrientation.x * tag.fieldOrientation.y);
        double t4 = 1.0 - 2.0 * (tag.fieldOrientation.y * tag.fieldOrientation.y + tag.fieldOrientation.z * tag.fieldOrientation.z);
        double yaw = Math.atan2(t3, t4);

        Vector2d[] points = {
                new Vector2d(0.25, 0.25),
                new Vector2d(0.5, -0.5),
                new Vector2d(-0.5, -0.5),
                new Vector2d(-0.25, 0.25),
        };

        for (int i = 0; i < 4; i++) {
//            Scale by size
            points[i] = points[i].times(-width);
//        Rotate
            points[i] = Rotation2d.fromDouble(yaw).times(points[i]);
//        Translate by x and y
            points[i] = points[i].plus(pos);
        }

        return points;
    }


    public static Pose2d getRobotPose(AprilTagDetection tag, OpenGLMatrix cameraPos) {
        AprilTagMetadata meta = tag.metadata;
        float tagX = meta.fieldPosition.get(0);
        float tagY = meta.fieldPosition.get(1);
        float tagZ = meta.fieldPosition.get(2);
        OpenGLMatrix tagR = new OpenGLMatrix(meta.fieldOrientation.toMatrix());
        OpenGLMatrix tagInField = OpenGLMatrix.identityMatrix()
                .translated(tagX, tagY, tagZ)
                .multiplied(tagR);

        OpenGLMatrix cameraInTagFrame = OpenGLMatrix.identityMatrix()
                .translated((float) tag.rawPose.x, (float) tag.rawPose.y, (float) tag.rawPose.z)
                .multiplied(new OpenGLMatrix(tag.rawPose.R))
                .inverted();

        OpenGLMatrix robotInCameraFrame = cameraPos.inverted();

        OpenGLMatrix robotInFieldFrame = tagInField
                .multiplied(cameraInTagFrame)
                .multiplied(robotInCameraFrame);

        Orientation orientation = Orientation.getOrientation(robotInFieldFrame, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        return new Pose2d(
                robotInFieldFrame.getTranslation().get(0),
                robotInFieldFrame.getTranslation().get(1),
                orientation.thirdAngle + Math.PI / 2
        );
    }

    public static Pose2d getTagFeildPose(AprilTagDetection tag) {
        AprilTagMetadata meta = tag.metadata;
        float tagX = meta.fieldPosition.get(0);
        float tagY = meta.fieldPosition.get(1);
        float tagZ = meta.fieldPosition.get(2);
        OpenGLMatrix tagR = new OpenGLMatrix(meta.fieldOrientation.toMatrix());
        OpenGLMatrix tagInField = OpenGLMatrix.identityMatrix()
                .translated(tagX, tagY, tagZ)
                .multiplied(tagR);

        Orientation orientation = Orientation.getOrientation(tagInField, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        return new Pose2d(
                tagInField.getTranslation().get(0),
                tagInField.getTranslation().get(1),
                orientation.thirdAngle + Math.PI / 2
        );
    }


    public static Pose2d getObjectPose(Pose2d robotPose, AprilTagDetection tag, OpenGLMatrix cameraInRobotFrame) {
        float robotX = (float) robotPose.position.x;
        float robotY = (float) robotPose.position.y;
        float robotZ = 0;
        OpenGLMatrix robotR = new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
                AngleUnit.RADIANS, 0, 0, (float) (robotPose.heading.toDouble() - (Math.PI / 2)), 0)
                .getRotationMatrix();
        OpenGLMatrix robotInField = OpenGLMatrix.identityMatrix()
                .translated(robotX, robotY, robotZ)
                .multiplied(robotR);

        OpenGLMatrix tagInCameraFrame = OpenGLMatrix.identityMatrix()
                .translated((float) tag.rawPose.x, (float) tag.rawPose.y, (float) tag.rawPose.z)
                .multiplied(new OpenGLMatrix(tag.rawPose.R));

        OpenGLMatrix robotInFieldFrame =
                robotInField
                        .multiplied(cameraInRobotFrame)
                        .multiplied(tagInCameraFrame);

        Orientation orientation = Orientation.getOrientation(robotInFieldFrame, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        return new Pose2d(
                robotInFieldFrame.getTranslation().get(0),
                robotInFieldFrame.getTranslation().get(1),
                orientation.thirdAngle
        );
    }

    public void drawApriltags(TelemetryPacket p) {
        Canvas fieldOverlay = p.fieldOverlay();

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
    }
}
