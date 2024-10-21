package com.userjhansen.automap;

import com.acmerobotics.roadrunner.Pose2d;

public class LocationMath {
    static public Pose2d getRobotLocationFromRobotPart(Pose2d partFinalLocation, Pose2d robotPartOffset) {
        double robotRotation = partFinalLocation.heading.minus(robotPartOffset.heading);

        double rotatedOffsetX = robotPartOffset.position.x * Math.cos(partFinalLocation.heading.real) + robotPartOffset.position.y * Math.sin(partFinalLocation.heading.real);
        double rotatedOffsetY = robotPartOffset.position.x * Math.sin(partFinalLocation.heading.real) - robotPartOffset.position.y * Math.cos(partFinalLocation.heading.real);

        double robotPartX = partFinalLocation.position.x - rotatedOffsetX;
        double robotPartY = partFinalLocation.position.y - rotatedOffsetY;

        return new Pose2d(
                robotPartX,
                robotPartY,
                robotRotation
        );
    }
}
