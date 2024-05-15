package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config

public class LiftArmAssembly {
    private final Lift liftL;
    private final Lift liftR;
    private final ServoMultiState elbowL;
    private final ServoMultiState wristL;
    private final ServoToggle claw;
    private final ServoToggle flap;
    public ArmTarget target = ArmTarget.Waiting;
    public double liftTargetHeight = 9;
    public static double ENDGAME_LIFT_HEIGHT = 7.9;
    private ArmTarget oldTarget = null;
    private Deadline updateTimeout;

    public LiftArmAssembly(HardwareMap hardwareMap) {
        liftL = new Lift(hardwareMap.get(DcMotorEx.class, "liftL"), DcMotor.Direction.FORWARD);
        liftR = new Lift(hardwareMap.get(DcMotorEx.class, "liftR"), DcMotor.Direction.REVERSE);
        elbowL = new ServoMultiState(hardwareMap, "elbow", new double[]{0, 0.8, 0.2});
        wristL = new ServoMultiState(hardwareMap, "wrist", new double[]{0.52, 0, 0.00});
        claw = new ServoToggle(hardwareMap, "claw", 0.35, 0.7);
        flap = new ServoToggle(hardwareMap, "flap", 1.0, 0.3);
    }

    public void update(boolean leftPixel, boolean rightPixel) {
        Logging.LOG("liftHeight", liftTargetHeight);
        Logging.LOG("lift-T-I", liftL.targetHeight);
        Logging.LOG("lift-T-T", liftL.liftMotor.getTargetPosition());
        Logging.LOG("liftL-P", liftL.liftMotor.getCurrentPosition());
        Logging.LOG("liftR-P", liftR.liftMotor.getCurrentPosition());

        Logging.LOG("elbowL", elbowL.get());
        Logging.LOG("wristL", wristL.get());
        Logging.LOG("claw", claw.get());
        Logging.LOG("flap", flap.get());

        if (target == ArmTarget.Waiting) {
            if (leftPixel && rightPixel && updateTimeout == null) {
                updateTimeout = new Deadline(1500, TimeUnit.MILLISECONDS);
            } else if ((!leftPixel || !rightPixel) && updateTimeout != null) {
                updateTimeout = null;
            }
        }

        if (target == ArmTarget.LiftRaised) setLiftPosition(liftTargetHeight);

        if (updateTimeout != null && updateTimeout.hasExpired()) {
            target = target.next;
        }

        if (oldTarget != target) {
            oldTarget = target;
            if (target.timeOut != -1) {
                updateTimeout = new Deadline((long) target.timeOut, java.util.concurrent.TimeUnit.MILLISECONDS);
            } else {
                updateTimeout = null;
            }

            switch (target) {
                case Caught:
                case Waiting:
                    setLiftPosition(Lift.liftBase);
                    setElbowPosition(2);
                    setWristPosition(false);
                    claw.changeTo(false);
                    flap.changeTo(true);
                    break;
                case Nab:
                    setLiftPosition(Lift.liftBase);
                    setElbowPosition(false);
                    setWristPosition(false);
                    claw.changeTo(false);
                    flap.changeTo(true);
                    break;
                case Primed:
                    setLiftPosition(Lift.liftBase);
                    setElbowPosition(false);
                    setWristPosition(false);
                    claw.changeTo(true);
                    flap.changeTo(true);
                    break;
                case LiftRaised:
                    setLiftPosition(liftTargetHeight);
                    setElbowPosition(true);
                    setWristPosition(true);
                    claw.changeTo(true);
                    flap.changeTo(false);
                    break;
                case Drop:
                    setLiftPosition(liftTargetHeight);
                    setElbowPosition(true);
                    setWristPosition(2);
                    claw.changeTo(false);
                    flap.changeTo(false);
                    break;
                case Return:
                    setLiftPosition(Lift.liftBase);
                    setElbowPosition(true);
                    setWristPosition(true);
                    claw.changeTo(false);
                    flap.changeTo(true);
                    break;
            }
        }
    }

    public void endgame(Boolean pull) {
        setLiftPosition(pull ? ENDGAME_LIFT_HEIGHT : Lift.liftBase);
        setElbowPosition(true);
    }

    private void setLiftPosition(double height) {
        liftL.setHeight(height);
        liftR.setHeight(height);
    }

    private void setElbowPosition(boolean up) {
        elbowL.changeTo(up);
    }

    private void setElbowPosition(int state) {
        elbowL.changeTo(state);
    }

    private void setWristPosition(boolean state) {
        wristL.changeTo(state);
    }

    private void setWristPosition(int state) {
        wristL.changeTo(state);
    }

    private final Deadline updateDeadline = new Deadline(25, java.util.concurrent.TimeUnit.MILLISECONDS);

    public void update(double up, double down) {
        if (updateDeadline.hasExpired()) {
            updateDeadline.reset();
            liftTargetHeight += up * 0.1;
            liftTargetHeight -= down * 0.1;
        }
    }
}
