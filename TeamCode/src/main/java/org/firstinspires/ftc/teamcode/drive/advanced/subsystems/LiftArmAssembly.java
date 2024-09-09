package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
public class LiftArmAssembly {
    private final Lift intakeSlides;
    private final ServoToggle intakeFlip;
    private final SelectiveIntakeMotor intakeMotor;

    private final Lift outtakeLift;
    private final ContinuousServoToggle outtakeServo;
    private final ServoMultiState fourBar;

    public ArmTarget target = ArmTarget.Retracted;
    public double intakeDistance = 10;
    public double outtakeDistance = 10;
    private ArmTarget oldTarget = null;
    private Deadline updateTimeout;

    public LiftArmAssembly(HardwareMap hardwareMap) {
        intakeSlides = new Lift(hardwareMap.get(DcMotorEx.class, "intakeSlide"), DcMotor.Direction.FORWARD, new PIDFCoefficients(1, 0, 0, 0));
        intakeFlip = new ServoToggle(hardwareMap, "intakeFlip", 0d, 1d); // Down state is on, up state is off
        intakeMotor = new SelectiveIntakeMotor(hardwareMap, "intakeMotor", "intakeColour", 0.8);

        outtakeLift = new Lift(hardwareMap.get(DcMotorEx.class, "outtakeLift"), DcMotor.Direction.FORWARD, new PIDFCoefficients(1, 0, 0, 0));
        outtakeServo = new ContinuousServoToggle(hardwareMap, "outtakeServo", 0d, 1d); // Off is off, on is forward (outtake)
        fourBar = new ServoMultiState(hardwareMap, "fourBar", new double[]{0, 1}); // State 0 is ready to pickup the inttaked sample, State 1 is ready to deploy
    }

    public void update() {
        Logging.LOG("LIFT STATE MACHINE:", this.target);

        this.intakeMotor.update();

        if (target == ArmTarget.Searching) {
            boolean intakeState = this.intakeMotor.get();
            if (intakeState && updateTimeout == null) { // Intake has a sample
                updateTimeout = new Deadline(500, TimeUnit.MILLISECONDS); // Make for sure sure that we have a sample
            } else if (!intakeState && updateTimeout != null) { // No sample in intake
                updateTimeout = null;
            }
        }

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
                case Retracted:
                case IntakeRetract:
                case OuttakeRetract:
                    this.intakeSlides.setDistance(0);
                    this.intakeFlip.update(false);
                    this.intakeMotor.active = false;

                    this.outtakeLift.setDistance(0);
                    this.outtakeServo.update(false);
                    this.fourBar.update(false);
                    break;

                case IntakeExtend:
                case Captured:
                    this.intakeSlides.setDistance(intakeDistance);
                    this.intakeFlip.update(false);
                    this.intakeMotor.active = false;

                    this.outtakeLift.setDistance(0);
                    this.outtakeServo.update(false);
                    this.fourBar.update(false);
                    break;

                case Searching:
                    this.intakeSlides.setDistance(intakeDistance);
                    this.intakeFlip.update(true);
                    this.intakeMotor.active = true;

                    this.outtakeLift.setDistance(0);
                    this.outtakeServo.update(false);
                    this.fourBar.update(false);
                    break;

                case InternalTransfer:
                    this.intakeSlides.setDistance(0);
                    this.intakeFlip.update(false);
                    this.intakeMotor.overrideMotor(0.2);

                    this.outtakeLift.setDistance(0);
                    this.outtakeServo.update(false);
                    this.fourBar.update(false);
                    break;

                case OuttakeExtend:
                case OuttakeReady:
                    this.intakeSlides.setDistance(0);
                    this.intakeFlip.update(false);
                    this.intakeMotor.overrideMotor(0);

                    this.outtakeLift.setDistance(outtakeDistance);
                    this.outtakeServo.update(false);
                    this.fourBar.update(true);
                    break;

                case Outtake:
                    this.intakeSlides.setDistance(0);
                    this.intakeFlip.update(false);
                    this.intakeMotor.overrideMotor(0);

                    this.outtakeLift.setDistance(outtakeDistance);
                    this.outtakeServo.update(true);
                    this.fourBar.update(true);
                    break;
            }
        }
    }

    public void reset() {
        this.target = this.target.safeBack;
    }
}
