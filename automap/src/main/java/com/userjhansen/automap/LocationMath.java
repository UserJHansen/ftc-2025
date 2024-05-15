package com.userjhansen.automap;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class LocationMath {
    static public Pose2d getRobotLocationFromRobotPart(Pose2d partFinalLocation, Pose2d robotPartOffset) {
        double robotRotation = partFinalLocation.getHeading() - robotPartOffset.getHeading();

        double rotatedOffsetX = robotPartOffset.getX() * Math.cos(partFinalLocation.getHeading()) + robotPartOffset.getY() * Math.sin(partFinalLocation.getHeading());
        double rotatedOffsetY = robotPartOffset.getX() * Math.sin(partFinalLocation.getHeading()) - robotPartOffset.getY() * Math.cos(partFinalLocation.getHeading());

        double robotPartX = partFinalLocation.getX() - rotatedOffsetX;
        double robotPartY = partFinalLocation.getY() - rotatedOffsetY;

        return new Pose2d(
                robotPartX,
                robotPartY,
                robotRotation
        );
    }


    //    Takes the robot pose, offset of the base position of the lift, the angle the lift is at and the number of inches the lift is extended
//    Returns the pose of the lift
//    The liftVerticalAngle is the angle of the lift from the ground, x in the diagram:
//      /|
//     / |
//    /x |
//    The lift rotation is calculated from the lift offset heading and robotPose
    public static Pose2d calculateLiftPose(Pose2d robotPose, Pose2d liftOffset, double liftVerticalAngle, double liftExtension) {
        double liftRotation = robotPose.getHeading() + liftOffset.getHeading();
        double liftX = robotPose.getX() + liftOffset.getX() + (liftExtension * Math.cos(liftVerticalAngle));
        double liftY = robotPose.getY() + liftOffset.getY() + (liftExtension * Math.sin(liftVerticalAngle));

        return new Pose2d(liftX, liftY, liftRotation);
    }

    //    Calculates the amount the lift has to be extended in inches to get the required height
    public static double calculateLiftExtension(double liftFloorOffset, double liftVerticalAngle, double desiredHeight) {
        return (desiredHeight - liftFloorOffset) / Math.cos(liftVerticalAngle);
    }
}
