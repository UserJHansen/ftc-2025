package com.userjhansen.automap.Maps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.PartType;

public class InsideOne implements Map {
    public Pose2d startPosition = new Pose2d(-35, -60, -Math.PI / 2);

    public static Pose2d depositPosition = new Pose2d(-60, -50, -(Math.PI / 16) + (Math.PI/2));

    public static AutoPart[] startParts = {
            new AutoPart(PartType.STRAFE, new Pose2d(-9, -32,0)),
            new AutoPart(PartType.ACTION, 0),

            new AutoPart(PartType.SPLINE_TO, new Pose2d(-48, -40, Math.PI/2), Math.PI),
            new AutoPart(PartType.STRAFE_TO, new Pose2d(-48, -35, Math.PI/2)),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE_TO, depositPosition),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.STRAFE_TO, new Pose2d(-59, -35, Math.PI/2)),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE_TO, depositPosition),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.STRAFE_TO, new Pose2d(-54, -24, -Math.PI)),
            new AutoPart(PartType.STRAFE, new Pose2d(-57, -24, -Math.PI)),
            new AutoPart(PartType.ACTION, 1),
            new AutoPart(PartType.STRAFE_TO, depositPosition),
            new AutoPart(PartType.ACTION, 2),

            new AutoPart(PartType.STRAFE_TO, new Pose2d(-50, -12, 0)),
            new AutoPart(PartType.STRAFE_TO, new Pose2d(-22, -12, 0)),
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
