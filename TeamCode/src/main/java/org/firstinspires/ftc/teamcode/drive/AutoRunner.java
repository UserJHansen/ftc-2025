package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.OutsideOne;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;

@Config
public class AutoRunner extends Robot {
    public static double distanceThreshold = 10;
    public static double angleThreshold = Math.toRadians(45);
    public RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    private boolean hasLocked = false;
    private boolean inPosition = false;

    public AutoRunner(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, true);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void updateInit() {
        super.updateInit();
        hasLocked |= adjustedPose;

//        UPDATE Light state
        if (!hasLocked) {
//            statusLEDs.changeTo(RevBlinkinLedDriver.BlinkinPattern.RED);
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
//            statusLEDs.changeTo(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            return;
        }

        if (!inPosition) {
            inPosition = true;
//            camera.selectCameraTwo(true);
        }
    }

    @Override
    public void update() {
        super.update();

        if (!hasLocked) {
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
