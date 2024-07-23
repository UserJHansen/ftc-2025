package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class GyroLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double BIG_WHEEL_RADIUS = 1.181102; // in
    public static double SMALL_WHEEL_RADIUS = 0.6889764; // in

    public static double LATERAL_DISTANCE = 7.83867; // in; distance between the left and right wheels
    // 8.2806825
    public static double FORWARD_OFFSET = -5.9; // in; offset of the lateral wheel
    public static double X_OFFSET = 0;
    public static double X_MULTIPLIER = 1.01; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9671641791044776; // Multiplier in the Y direction

    private final Encoder leftEncoder;
    private final Encoder frontEncoder;
    private final IMU imu;

    private final List<Integer> lastEncPositions;
    private final List<Integer> lastEncVels;


    public GyroLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(X_OFFSET, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(X_OFFSET + FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder"));
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks, double WHEEL_RADIUS) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos, BIG_WHEEL_RADIUS) * X_MULTIPLIER,
                encoderTicksToInches(frontPos, SMALL_WHEEL_RADIUS) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel, BIG_WHEEL_RADIUS) * X_MULTIPLIER,
                encoderTicksToInches(frontVel, SMALL_WHEEL_RADIUS) * Y_MULTIPLIER
        );
    }

    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
