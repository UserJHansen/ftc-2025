package org.firstinspires.ftc.teamcode.staticData;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, 0);

    public static boolean isRedAlliance = false;
    public static boolean isInit = true;

    public static boolean splitControls = true;

    public static boolean shouldHallucinate = false;
}
