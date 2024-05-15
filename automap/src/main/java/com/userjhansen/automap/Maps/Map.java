package com.userjhansen.automap.Maps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.userjhansen.automap.AutoPart;

public interface Map {
    public Pose2d getStartPosition();

    public AutoPart[] getStartParts();

    public AutoPart[] getBackdropParts();
}
