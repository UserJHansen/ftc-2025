package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;

import java.util.ArrayList;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

@Config
public class RollbackLocalizer implements Localizer {
    Localizer sourceLocalizer;

    Map<Long, Pose2d> poseDiffs = new TreeMap<>();

    public RollbackLocalizer(Localizer sourceLocalizer) {
        this.sourceLocalizer = sourceLocalizer;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return this.sourceLocalizer.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.sourceLocalizer.setPoseEstimate(pose2d);
    }

    public void newDelayedVisionPose(@NonNull Pose2d newPose, double pipelineLatency) {
        // Reset the location to the new pose then apply the latency worth of diffs
        long currentTime = System.currentTimeMillis();

        Pose2d currentPose = newPose;
        ArrayList<Long> deleteTimestamps = new ArrayList<>();
        for (long timeStamp : poseDiffs.keySet()) {
            if (timeStamp + pipelineLatency < currentTime) {
                // Diff is old, delete it
                deleteTimestamps.add(timeStamp);
            } else {
                // Diff is within the latency window, use it to correct the current location
                currentPose = currentPose.plus(Objects.requireNonNull(poseDiffs.get(timeStamp)));
            }
        }
        for (long timestamp : deleteTimestamps) {
            poseDiffs.remove(timestamp);
        }
        // Apply the new location
        this.setPoseEstimate(currentPose);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return this.sourceLocalizer.getPoseVelocity();
    }

    @Override
    public void update() {
        Pose2d previousPose = this.sourceLocalizer.getPoseEstimate();
        this.sourceLocalizer.update();
        Pose2d currentPose = this.sourceLocalizer.getPoseEstimate();
        Pose2d poseDiff = currentPose.minus(previousPose);

        long currentTime = System.currentTimeMillis();

        if (poseDiffs.containsKey(currentTime)) {
            Logging.LOG("[ERROR] The main loop is running faster than 1000fps, meaning that the pose diff is losing state");

            poseDiffs.computeIfPresent(currentTime, (k, previousDiff) -> previousDiff.plus(poseDiff));
        } else {
            poseDiffs.put(currentTime, poseDiff); // New diff for current ms
        }
    }
}
