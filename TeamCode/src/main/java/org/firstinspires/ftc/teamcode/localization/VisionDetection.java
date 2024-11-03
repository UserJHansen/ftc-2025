package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;

@Config
public class VisionDetection {
    public static Double stdDevLimit = 0.05;
    public Limelight3A limelight;
    long lastResult = 0;
    long iteration = 0;
    public VisionDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update(RollbackLocalizer localizer, TelemetryPacket packet) {
        iteration += 1;

        if (iteration % 20 == 0) {
            limelight.updateRobotOrientation(AngleUnit.DEGREES.fromRadians(localizer.currentPose.heading.toDouble()));
        }

        if (iteration % 200 == 0) {
            LLStatus status = limelight.getStatus();
            Logging.LOG("(limelight) Temp", status.getTemp());
            Logging.LOG("(limelight) CPU", status.getCpu());
            Logging.LOG("(limelight) FPS", status.getFps());
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStddevMt2()[0] < stdDevLimit && result.getControlHubTimeStamp() > lastResult) {
            lastResult = result.getControlHubTimeStamp();

            // Access general information
            Pose3D botpose = result.getBotpose_MT2();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            double staleness = result.getStaleness();

            Pose2d robotPose = new Pose2d(
                    DistanceUnit.INCH.fromMeters(botpose.getPosition().x),
                    DistanceUnit.INCH.fromMeters(botpose.getPosition().y),
                    AngleUnit.RADIANS.fromDegrees(botpose.getOrientation().getYaw()));

            Logging.DEBUG("(limelight) confidence", result.getStddevMt2()[0]);
            Logging.LOG("(limelight) latency", captureLatency + targetingLatency + parseLatency + staleness);
            Logging.LOG("(limelight) Last Pose", robotPose);

            double x = robotPose.position.x;
            double y = robotPose.position.y;
            if (x < -72 || x > 72 || y > 72 || y < -72) {
//                Out of bounds
//                This is due to MT2 issues with rotation
                Logging.LOG("OOB Location");
                if (PoseStorage.isInit) {
                    limelight.updateRobotOrientation(AngleUnit.DEGREES.fromRadians(robotPose.heading.toDouble() + Math.PI));
                    localizer.setCurrentPose(new Pose2d(0, 0, robotPose.heading.toDouble() + Math.PI));
                }
                return;
            }

            packet.fieldOverlay().setStroke("#00ffff");
            Drawing.drawRobot(packet.fieldOverlay(), robotPose);

            localizer.newDelayedVisionPose(robotPose, captureLatency + targetingLatency + parseLatency + staleness);
        } else {
            Logging.LOG("(limelight) Status", "No data available");
        }
    }
}
