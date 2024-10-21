package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
public class Lift {
    public static double ticksPerInch = 47.7419354839;

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

    public interface ResetCheck {
        boolean call();
    }

    private ResetCheck endLimit;
    Deadline endpointTimeout;
    public void reset(ResetCheck resetCondition) {
        this.endLimit = resetCondition;
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.liftMotor.setPower(-0.25);
        endpointTimeout = new Deadline(2, TimeUnit.SECONDS);
    }

    public void update() {
        if (endpointTimeout != null) {
            if (endpointTimeout.hasExpired() || this.endLimit.call() ) { // High is true, disconnected
                endpointTimeout = null;
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setTargetPosition(0);
                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                liftMotor.setPower(1);
                setDistance(targetDistance);
            }
        }
    }

    public double currentPosition() {
        return liftMotor.getCurrentPosition() / ticksPerInch;
    }

    public void setDistance(double distance) {
        targetDistance = distance;
        if (endpointTimeout == null) { // Still calibrating
            targetDistance = distance;
            int target = (int) (distance * ticksPerInch);
            liftMotor.setTargetPosition(target);
        }
    }
}
