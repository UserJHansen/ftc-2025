package com.userjhansen.automap;

import com.acmerobotics.roadrunner.Pose2d;
import com.userjhansen.automap.Maps.Map;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoPart {
    public PartType type;
    private final Pose2d pose;
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

    public Pose2d getPose() {
        if (offset == null) {
            return pose;
        } else {
            return LocationMath.getRobotLocationFromRobotPart(pose, offset);
        }
    }


    public static AutoPart[] makeFullAutoList(Map map) {
        ArrayList<AutoPart> parts = new ArrayList<>();

        parts.add(new AutoPart(PartType.CHANGE_LIGHT, 0));
        parts.add(new AutoPart(PartType.STRAFE, map.getSpecimenPosition()));
        parts.add(new AutoPart(PartType.ACTION, 0));

        for (AutoPart[] partList: map.getIntakeParts()) {
            parts.addAll(Arrays.asList(partList));
            parts.add(new AutoPart(PartType.ACTION, 1));
            parts.addAll(Arrays.asList(map.getDepositParts()));
            parts.add(new AutoPart(PartType.ACTION, 2));
        }

        parts.addAll(Arrays.asList(map.getParkParts()));

        parts.add(new AutoPart(PartType.CHANGE_LIGHT, 1));

        return parts.toArray(new AutoPart[0]);
    }
}