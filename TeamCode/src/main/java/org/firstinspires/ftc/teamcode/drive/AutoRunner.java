package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;

@Config
public class AutoRunner extends Robot {
    public static double distanceThreshold = 10;
    public static double angleThreshold = Math.toRadians(45);

    public AutoRunner(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, true);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private boolean hasLocked = false;
    private boolean inPosition = false;

    @Override
    public void updateInit() {
        super.updateInit();
        hasLocked |= adjustedPose;

//        UPDATE Light state
        if (!hasLocked) {
            statusLEDs.changeTo(RevBlinkinLedDriver.BlinkinPattern.RED);
            return;
        }

        boolean isRed = driveBase.getPoseEstimate().getY() < 0;
        boolean isInside = driveBase.getPoseEstimate().getX() > 0;

        Pose2d startingPos = isInside ? new InsideOne().getStartPosition() : new OutsideOne().getStartPosition();
        startingPos = new Pose2d(startingPos.getX(), startingPos.getY() * (isRed ? 1 : -1), startingPos.getHeading() + (isRed ? 0 : Math.PI));

        Pose2d currentDiff = startingPos.minus(driveBase.getPoseEstimate());
        Logging.LOG("Distance", currentDiff.vec().distTo(new Vector2d()));
        Logging.LOG("Heading Diff", Math.abs(currentDiff.getHeading()));
        if (currentDiff.vec().distTo(new Vector2d()) > distanceThreshold || Math.abs(currentDiff.getHeading()) > angleThreshold) {
            statusLEDs.changeTo(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            return;
        }

        if (!inPosition) {
            inPosition = true;
            camera.selectCameraTwo(true);
        }

        switch (VisionDetection.position) {
            case 1:
                statusLEDs.changeTo(List.of(
                        RevBlinkinLedDriver.BlinkinPattern.GREEN,
                        RevBlinkinLedDriver.BlinkinPattern.VIOLET
                ));
                break;
            case 2:
                statusLEDs.changeTo(List.of(
                        RevBlinkinLedDriver.BlinkinPattern.GREEN,
                        RevBlinkinLedDriver.BlinkinPattern.BLUE
                ));
                break;
            case 3:
                statusLEDs.changeTo(List.of(
                        RevBlinkinLedDriver.BlinkinPattern.GREEN,
                        RevBlinkinLedDriver.BlinkinPattern.YELLOW
                ));
                break;
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    @Override
    public void update() {
        super.update();
        camera.selectCameraTwo(true);

        if (!hasLocked) {
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return;
        }

        switch (VisionDetection.position) {
            case 1:
                statusLEDs.changeTo(List.of(
                        currentPattern,
                        RevBlinkinLedDriver.BlinkinPattern.VIOLET
                ));
                break;
            case 2:
                statusLEDs.changeTo(List.of(
                        currentPattern,
                        RevBlinkinLedDriver.BlinkinPattern.BLUE
                ));
                break;
            case 3:
                statusLEDs.changeTo(List.of(
                        currentPattern,
                        RevBlinkinLedDriver.BlinkinPattern.YELLOW
                ));
                break;
        }
    }
}
