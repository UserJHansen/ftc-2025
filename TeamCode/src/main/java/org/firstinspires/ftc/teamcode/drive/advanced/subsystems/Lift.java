package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.userjhansen.automap.LocationMath;
import com.userjhansen.automap.RobotParams;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Config
public class Lift {
    public static double liftBase = 6;
    public static double ticksPerInch = 115;

    public static int hardLimit = 1600;

    public DcMotorEx liftMotor;
    public double targetHeight;

    public Lift(DcMotorEx liftMotor, DcMotorSimple.Direction direction) {
        this.liftMotor = liftMotor;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(1);
        liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, DriveConstants.liftPIDF);

        liftMotor.setDirection(direction);
    }

    public void setHeight(double height) {
        targetHeight = height;
        int target = (int) (LocationMath.calculateLiftExtension(liftBase, Math.toRadians(RobotParams.liftAngleVertical), height) * ticksPerInch);
        liftMotor.setTargetPosition(Math.max(Math.min(target, hardLimit), 0));
    }
}
