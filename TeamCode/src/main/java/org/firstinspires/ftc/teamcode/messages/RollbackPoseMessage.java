package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class RollbackPoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;
    public double latency;

    public RollbackPoseMessage(Pose2d pose, double latency) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
        this.latency = latency;
    }
}

