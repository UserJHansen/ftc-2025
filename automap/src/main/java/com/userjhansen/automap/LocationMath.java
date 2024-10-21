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
}
