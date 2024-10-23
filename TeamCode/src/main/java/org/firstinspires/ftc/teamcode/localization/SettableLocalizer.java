package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;

public interface SettableLocalizer extends Localizer {
    void setCurrentPose(Pose2d pose);
}
