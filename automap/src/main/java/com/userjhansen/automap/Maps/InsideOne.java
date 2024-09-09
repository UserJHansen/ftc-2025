package com.userjhansen.automap.Maps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.PartType;

public class InsideOne implements Map {
    public Pose2d startFloorPixel = new Pose2d(15, -60, Math.PI / 2);

    public static AutoPart[] floorPixelParts = {
            new AutoPart(PartType.FORWARD, 9),
            new AutoPart(PartType.STRAFE, new Pose2d(15, -50)),
            new AutoPart(PartType.SPLINE_TO, new Pose2d(1, -28, Math.PI), Math.PI),
            new AutoPart(PartType.WAIT, 3),
            new AutoPart(PartType.ACTION, 0),
            new AutoPart(PartType.STRAFE, new Pose2d(15, -28)),

    };

    public static AutoPart[] backdropPixelParts = {
            new AutoPart(PartType.STRAFE, new Pose2d(58, -29, Math.PI)),
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
