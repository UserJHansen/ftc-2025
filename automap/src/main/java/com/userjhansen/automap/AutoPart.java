package com.userjhansen.automap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.Maps.Map;

import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class AutoPart {
    public PartType type;
    private Pose2d pose;
    @Nullable
    private Pose2d offset;

    public double value;

    public AutoPart(PartType type, Pose2d pose) {
        this.type = type;
        this.pose = pose;
        this.value = 0;
    }

    public AutoPart(PartType type, Pose2d pose, double value) {
        this.type = type;
        this.pose = pose;
        this.value = value;
    }

    public AutoPart(PartType type, Pose2d pose, Pose2d offset) {
        this.type = type;
        this.pose = pose;
        this.offset = offset;
        this.value = 0;
    }

    public AutoPart(PartType type, Pose2d pose, Pose2d offset, double value) {
        this.type = type;
        this.pose = pose;
        this.offset = offset;
        this.value = value;
    }

    public AutoPart(PartType type, double value) {
        this.type = type;
        this.pose = new Pose2d(0, 0, 0);
        this.value = value;
    }

    public Pose2d modified(double yMult, double headingMult) {
        Pose2d pose = getPose();
        return new Pose2d(pose.getX(), pose.getY() * yMult, pose.getHeading() * headingMult);
    }

    public Pose2d getPose() {
        if (offset == null) {
            return pose;
        } else {
            return LocationMath.getRobotLocationFromRobotPart(pose, offset);
        }
    }


    public static AutoPart[] makeFullAutoList(Map map, boolean isInside) {
        ArrayList<AutoPart> parts = new ArrayList<>();

        parts.add(new AutoPart(PartType.CHANGE_LIGHT, 0));
        parts.addAll(Arrays.asList(map.getStartParts()));

        parts.add(new AutoPart(PartType.CHANGE_LIGHT, 1));
//        GAME SPECIFIC CODE HERE
        if (!isInside) {
            parts.addAll(Arrays.asList(
                    new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(-32, -11, Math.PI)),
                    new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(34, -11, Math.PI))
            ));
        }

        parts.addAll(Arrays.asList(map.getBackdropParts()));

        parts.add(new AutoPart(PartType.CHANGE_LIGHT, 2));
//        MORE GAME SPECIFIC STUFF
        AutoPart[] parkingOutsideParts = new AutoPart[]{
                new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(48, -11, Math.PI)),
                new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(60, -11, Math.PI)),
        };
        AutoPart[] parkingInsideParts = new AutoPart[]{
                new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(48, -57, Math.PI)),
                new AutoPart(PartType.SPLINE_CONSTANT, new Pose2d(60, -60, Math.PI)),
        };
        parts.addAll(Arrays.asList(isInside ? parkingInsideParts : parkingOutsideParts));

        return parts.toArray(new AutoPart[0]);
    }
}