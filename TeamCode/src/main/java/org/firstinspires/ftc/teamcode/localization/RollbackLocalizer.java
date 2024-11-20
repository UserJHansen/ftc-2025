package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.RollbackPoseMessage;
import org.firstinspires.ftc.teamcode.staticData.Logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

public class RollbackLocalizer implements SettableLocalizer {
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    public Pose2d currentPose = new Pose2d(0, 0, 0);
    public Pose2d oldestPose = new Pose2d(0, 0, 0); // Used in path history drawing
    Localizer sourceLocalizer;
    private boolean settable;

    Map<Long, Twist2d> poseDiffs = new TreeMap<>();

    public RollbackLocalizer(SettableLocalizer sourceLocalizer) {
        this.sourceLocalizer = sourceLocalizer;
        settable = true;
    }

    public RollbackLocalizer(Localizer sourceLocalizer) {
        this.sourceLocalizer = sourceLocalizer;
        settable = false;
    }

    public void newDelayedVisionPose(@NonNull Pose2d newPose, double pipelineLatency) {
        // Reset the location to the new pose then apply the latency worth of diffs
        long currentTime = System.currentTimeMillis();

        Pose2d currentPose = newPose;
        Pose2d previousPose = oldestPose;
        ArrayList<Long> deleteTimestamps = new ArrayList<>();
        boolean encounteredNewData = false;
        for (long timeStamp : poseDiffs.keySet()) {
            if (timeStamp + pipelineLatency < currentTime) {
                // Diff is old, delete it
                deleteTimestamps.add(timeStamp);
                previousPose = previousPose.plus(Objects.requireNonNull(poseDiffs.get(timeStamp)));
            } else {
                currentPose = currentPose.plus(Objects.requireNonNull(poseDiffs.get(timeStamp)));
            }
        }
        for (long timestamp : deleteTimestamps) {
            poseDiffs.remove(timestamp);
        }

        Logging.LOG("POSE_CHANGE", newPose.minus(previousPose));
        // Apply the new location
        FlightRecorder.write("ROLLBACK_NEW_POSE", new RollbackPoseMessage(currentPose, pipelineLatency));
        this.setCurrentPose(currentPose);
    }

    @Override
    public void setCurrentPose(Pose2d currentPose) {
        this.poseDiffs = new HashMap<>();
        this.currentPose = currentPose;
        this.oldestPose = currentPose;
        if (settable)
            ((SettableLocalizer) this.sourceLocalizer).setCurrentPose(currentPose);
    }

    @Override
    public Twist2dDual<Time> update() {
        Twist2dDual<Time> localizerUpdate = this.sourceLocalizer.update();
        currentPose = currentPose.plus(localizerUpdate.value());

        long currentTime = System.currentTimeMillis();

        if (poseDiffs.containsKey(currentTime)) {
            Logging.LOG("[ERROR] The main loop is running faster than 1000fps, meaning that the pose diff is losing state");
        } else {
            poseDiffs.put(currentTime, localizerUpdate.value()); // New diff for current ms
        }

        estimatedPoseWriter.write(new PoseMessage(currentPose));

        return localizerUpdate;
    }

    public void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseDiffs.size()];
        double[] yPoints = new double[poseDiffs.size()];

        int i = 0;
        Pose2d pose = oldestPose;
        for (Twist2d diff : poseDiffs.values()) {
            pose = pose.plus(diff);
            xPoints[i] = pose.position.x;
            yPoints[i] = pose.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, currentPose);
    }
}
