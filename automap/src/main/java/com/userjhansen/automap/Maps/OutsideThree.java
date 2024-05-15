package com.userjhansen.automap.Maps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.PartType;
import com.userjhansen.automap.RobotParams;

public class OutsideThree implements Map {
    public Pose2d startFloorPixel = new Pose2d(-35, -60, Math.PI / 2);

    public static AutoPart[] floorPixelParts = {
            new AutoPart(PartType.FORWARD, 10),
            new AutoPart(PartType.STRAFE, new Pose2d(-37, -50)),
            new AutoPart(PartType.SPLINE_TO, new Pose2d(-24, -28, 0), RobotParams.intakeOffset, 0),
            new AutoPart(PartType.WAIT, 3),
            new AutoPart(PartType.ACTION, 0),
            new AutoPart(PartType.STRAFE, new Pose2d(-39, -28)),
            new AutoPart(PartType.STRAFE_TO, new Pose2d(-39, -11, Math.PI)),

    };

    public static AutoPart[] backdropPixelParts = {
            new AutoPart(PartType.STRAFE_TO, new Pose2d(58, -41, Math.PI), RobotParams.liftOffset),
            new AutoPart(PartType.WAIT, 3),
            new AutoPart(PartType.ACTION, 1),
    };


    @Override
    public Pose2d getStartPosition() {
        return startFloorPixel;
    }

    @Override
    public AutoPart[] getStartParts() {
        return floorPixelParts;
    }

    @Override
    public AutoPart[] getBackdropParts() {
        return backdropPixelParts;
    }
}
