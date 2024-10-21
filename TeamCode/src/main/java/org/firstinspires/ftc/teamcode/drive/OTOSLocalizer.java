package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OTOSLocalizer implements Localizer {
    public static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 180);
    public static double linearScalar = 1.0;
    public static double angularScalar = 1.0;

    SparkFunOTOS otos;

    public OTOSLocalizer(HardwareMap hardwareMap) {
        this.otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        if (!otos.begin()) {
            throw new RuntimeException("OTOS is not connected correctly");
        }

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(offset);
        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        otos.calibrateImu();
        otos.resetTracking();
    }

    Pose2d poseEstimate = new Pose2d();
    Pose2d poseVelocity = new Pose2d();

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
        otos.setPosition(new SparkFunOTOS.Pose2D(pose2d.getX(), pose2d.getY(), pose2d.getHeading()));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        SparkFunOTOS.Pose2D sparkfunPose = otos.getPosition();
        poseEstimate = new Pose2d(sparkfunPose.x, sparkfunPose.y, sparkfunPose.h);

        SparkFunOTOS.Pose2D velocityPose = otos.getVelocity();
        poseVelocity = new Pose2d(velocityPose.x, velocityPose.y, velocityPose.h);
    }
}
