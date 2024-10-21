package com.userjhansen.automap.Maps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.PartType;

public class OutsideOne implements Map {
    public Pose2d startPosition = new Pose2d(15, -60, -Math.PI / 2);

    public static Pose2d depositPosition = new Pose2d(-46, -58, Math.PI / 16);

    public static AutoPart[] startParts = {
            new AutoPart(PartType.STRAFE, new Pose2d(9, -32, 0)),
            new AutoPart(PartType.ACTION, 0), // Deposit specimen

            new AutoPart(PartType.STRAFE, new Pose2d(12, -40, 0)),
            new AutoPart(PartType.SPLINE_TO, new Pose2d(30, -40, Math.PI / 4)),
            new AutoPart(PartType.STRAFE_TO, new Pose2d(40, -34, Math.PI / 4)),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE, new Pose2d(0, -50, 0)),
            new AutoPart(PartType.SPLINE_TO, depositPosition, Math.PI),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.SPLINE_TO, new Pose2d(50, -30, (3*Math.PI) / 16), (5*Math.PI) / 16),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE, new Pose2d(0, -50, 0)),
            new AutoPart(PartType.SPLINE_TO, depositPosition, Math.PI),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.SPLINE_TO, new Pose2d(55, -27, (1*Math.PI) / 16), (1*Math.PI) / 16),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE, new Pose2d(0, -50, 0)),
            new AutoPart(PartType.SPLINE_TO, depositPosition, Math.PI),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.STRAFE, new Pose2d(0, -50, 0)),
            new AutoPart(PartType.STRAFE, new Pose2d(45, -25, 0)),
            new AutoPart(PartType.STRAFE, new Pose2d(45, -10, 0)),
            new AutoPart(PartType.STRAFE, new Pose2d(24, -10, 0)),
    };


    @Override
    public Pose2d getStartPosition() {
        return startPosition;
    }

    @Override
    public AutoPart[] getStartParts() {
        return startParts;
    }
}
