package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras.Camera;
import org.firstinspires.ftc.teamcode.drive.galahlib.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Config
public class CameraPoseAdjust {
    // If the robot is moving the pose could be messed up because of latency in the AprilTag pipeline
    public static double ROBOT_SPEED_CUTOFF = 1;
    public static double ROBOT_ROTATION_CUTOFF = 0.4;
    // All Poses must be within x inches of average to be considered for integrating with the previous pose
    // If not we ignore all results
    public static double POSE_DIFFERENCE_LIMIT = 5;
    public static double CLOSE_TAG_DISTANCE = 15;
    // FOV of camera, this is used in the weight calculation based on bearing
    // This is important because readings at the edge of the camera are less accurate
    public static double CAMERA_FOV = 70;
    // The weight that is given to the bearing
    public static double BEARING_WEIGHT = 0.5;
    // The weight given to OpenCVs inbuilt weight
    public static double CV_CONFIDENCE_WEIGHT = 0.5;
    Camera camera;
    SampleMecanumDrive robot;

    public CameraPoseAdjust(SampleMecanumDrive _robot, Camera _camera) {
        camera = _camera;
        robot = _robot;
    }

    public boolean update(boolean overrideDistance) {
        Vector2d vel;
        try {
            vel = robot.getPoseVelocity().vec();
        } catch (Exception e) {
            Logging.LOG("CAMERAPOSE", "DISABLED_NO_VELOCITY");
            return false;
        }
        if (Math.hypot(vel.getX(), vel.getY()) < ROBOT_SPEED_CUTOFF && robot.getExternalHeadingVelocity() < ROBOT_ROTATION_CUTOFF) {
            List<Pose2d> possiblePoses = new ArrayList<>();
            List<Double> weights = new ArrayList<>();

            possiblePoses.add(robot.getPoseEstimate());
            weights.add(0.75);

            for (AprilTagDetection tag : VisionDetection.detections) {
                if (tag.metadata != null && tag.id <= 10) {
                    possiblePoses.add(VisionMath.getRobotPose(tag, camera.getCameraPos()));

                    double distanceFromCenter = Math.abs(tag.ftcPose.bearing);
                    double bearingWeight = Math.log10((CAMERA_FOV / 2) - distanceFromCenter) / Math.log10(CAMERA_FOV / 2);

//                    We calculate a weight by trying to guess how accurate it would be
                    double weight = BEARING_WEIGHT * (bearingWeight)
                            + CV_CONFIDENCE_WEIGHT * tag.decisionMargin / 100;
                    weights.add(weight);
                }
            }
            Logging.DEBUG("CAMERAPOSE_POSSIBLE", possiblePoses.size());
            Logging.DEBUG("CAMERAPOSE_POSES", possiblePoses);
            Logging.DEBUG("CAMERAPOSE_WEIGHTS", weights);

            if (possiblePoses.size() == 1) {
                Logging.LOG("CAMERAPOSE", "DISABLED_NONEPOSSIBLE");
                return false;
            }

//            Normalise the weights
            double totalWeights = 1;
            for (Double w : weights) totalWeights += w;
            double n = 1 / totalWeights;

            Pose2d firstPoseDiff = possiblePoses.get(1).minus(VisionMath.getTagFeildPose(VisionDetection.detections.get(0)));
            double distance = Math.hypot(firstPoseDiff.getX(), firstPoseDiff.getY());
            Logging.LOG("First Distance", distance);
            if (possiblePoses.size() < 3 && distance > CLOSE_TAG_DISTANCE) {
                Logging.LOG("CAMERAPOSE", "DISABLED_TOO_LITTLE_DATA");
                return false;
            }

            Pose2d average = robot.getPoseEstimate().times(n);
            for (int i = 0; i < possiblePoses.size(); i++) {
                average = average.plus(possiblePoses.get(i).times(weights.get(i) * n));
            }

//            If we are detecting the canvas
            if (possiblePoses.size() == 3 && !overrideDistance && distance > CLOSE_TAG_DISTANCE) {
                // Make sure none of them are too far away
                for (Pose2d p : possiblePoses) {
                    Pose2d d = p.minus(average);
                    if (Math.hypot(d.getX(), d.getY()) > POSE_DIFFERENCE_LIMIT) {
                        Logging.LOG("CAMERAPOSE", "DISABLED_TOO_DIFFERENT");
                        return false;
                    }
                }
            }

            // It's good!
            Logging.LOG("CAMERAPOSE", "SUCCESS");
            Logging.LOG("Changing pose based on camera");
            Pose2d poseDifference = robot.getPoseEstimate().minus(average);
            Logging.DEBUG("Moving by", Math.hypot(poseDifference.getX(), poseDifference.getY()));
            robot.setPoseEstimate(average);
            return true;
        } else {
            Logging.DEBUG("CAMERAPOSE_POSSIBLE", 0);
            Logging.LOG("CAMERAPOSE", "DISABLED_MOVING");
            return false;
        }
    }
}
