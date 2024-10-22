package org.firstinspires.ftc.teamcode.drive;

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
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.SettableLocalizer;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

@Config
public class OTOSLocalizer implements SettableLocalizer {
    public static Pose2d offset = new Pose2d(0, 0, Math.toRadians(180));
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

        otos.setOffset(RRPoseToOTOSPose(offset));
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
        otos.setPosition(RRPoseToOTOSPose(pose));
    }

    Pose2d lastPose = new Pose2d(0,0,0);
    @Override
    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        Pose2d pose = OTOSPoseToRRPose(otosPose);
        Twist2d poseDifference = lastPose.minus(pose);
        lastPose = pose;

        FlightRecorder.write("OTOS_NEW_POSE", new PoseMessage(pose));

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {
                                poseDifference.line.x,
                                otosVel.x
                        }),
                        new DualNum<>(new double[] {
                                poseDifference.line.y,
                                otosVel.y
                        })
                ),
                new DualNum<>(new double[] {
                        poseDifference.angle,
                        otosVel.h
                })
        );
    }
}
