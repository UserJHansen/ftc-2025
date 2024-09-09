package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lift {
    public static double ticksPerInch = 115;

    public DcMotorEx liftMotor;
    public double targetDistance;

    public Lift(DcMotorEx liftMotor, DcMotorSimple.Direction direction, PIDFCoefficients pidf) {
        this.liftMotor = liftMotor;

        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(1);
        liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidf);

        liftMotor.setDirection(direction);
    }

    public void setDistance(double distance) {
        targetDistance = distance;
        int target = (int) (distance * ticksPerInch);
        liftMotor.setTargetPosition(target);
    }
}
