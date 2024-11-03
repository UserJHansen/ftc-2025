package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

@Config
public class OTOSLocalizer implements SettableLocalizer {
    public static Double offsetX = -0.505;
    public static Double offsetY = 0.2;
    // Average of 3 calibration runs:
    // Detected: 113.5293 Actual: 114.4882
    // Detected: 119.7631 Actual: 118.1102
    // Detected: 116.9569 Actual: 118.7008
    // = (1.0084462777 + 0.986198587 + 1.0149106209)/3
    public static double linearScalar = 1.0031851619;
    // Was off by 31 degrees after 10 rotations
    // 3600 / 3631 = 0.9914624071
    public static double angularScalar = 0.9914624071;

    SparkFunOTOS otos;
    Pose2d lastPose = new Pose2d(0, 0, 0);

    public OTOSLocalizer(HardwareMap hardwareMap) {
        this.otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        if (!otos.begin()) {
            throw new RuntimeException("OTOS is not connected correctly");
        }

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(RRPoseToOTOSPose(new Pose2d(offsetX, offsetY, 0)));
        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        otos.calibrateImu();
        otos.resetTracking();
    }

    public Pose2d OTOSPoseToRRPose(SparkFunOTOS.Pose2D pose) {
        return new Pose2d(pose.x, pose.y, pose.h);
    }

    public SparkFunOTOS.Pose2D RRPoseToOTOSPose(Pose2d pose) {
        return new SparkFunOTOS.Pose2D(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    @Override
    public void setCurrentPose(Pose2d pose) {
        lastPose = pose;
        otos.setPosition(RRPoseToOTOSPose(pose));
    }

    @Override
    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);
        Pose2d pose = OTOSPoseToRRPose(otosPose);
        Twist2d poseDifference = pose.minus(lastPose);
        lastPose = pose;

        FlightRecorder.write("OTOS_NEW_POSE", new PoseMessage(pose));

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{
                                poseDifference.line.x,
                                otosVel.x
                        }),
                        new DualNum<>(new double[]{
                                poseDifference.line.y,
                                otosVel.y
                        })
                ),
                new DualNum<>(new double[]{
                        poseDifference.angle,
                        otosVel.h
                })
        );
    }
}
