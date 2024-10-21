package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@Config
public class LiftArmAssembly {
    public final SelectiveIntakeMotor intakeMechanism;

    private final Lift outtakeSlides;
    private final ServoMultiState outtakeGrabber;
    private final ServoMultiState elbow;
    private final ServoMultiState wrist;
    private final RevTouchSensor outakeEnd;
    private final RevColorSensorV3 transferCradle;

    public ArmTarget target = ArmTarget.Retracted;
    public static double outtakeDistance = 27;
    private ArmTarget oldTarget = null;
    private Deadline updateTimeout;

    public static double P_Outake = 3;
    public static double TransferSpeed = 0.6;

    public LiftArmAssembly(HardwareMap hardwareMap) {
        intakeMechanism = new SelectiveIntakeMotor(hardwareMap);

        outtakeSlides = new Lift(hardwareMap.get(DcMotorEx.class, "outtakeSlides"), DcMotor.Direction.REVERSE, new PIDFCoefficients(P_Outake, 0, 0, 0));
        outtakeGrabber = new ServoMultiState(hardwareMap, "outtakeGrabber", new double[]{0.5, 1}); // Off is off, on is grabbing
        elbow = new ServoMultiState(hardwareMap, "elbow", new double[]{0.15, 1, 0.48, 0}); // Off is intakeTransfer, on is full out, 2 is specimen recieve/store, 3 is deflect for intake
        wrist = new ServoMultiState(hardwareMap, "wrist", new double[]{0.33, 0.15, 0.15, 0.4}); // Off is horizontal, on is vertical, 2 is specimen recieve/store
        outakeEnd = hardwareMap.get(RevTouchSensor.class, "outtakeLimit");
        transferCradle = hardwareMap.get(RevColorSensorV3.class, "transferCradle");
    }

    public void update() {
        Logging.LOG("LIFT STATE MACHINE", this.target);

        this.outtakeSlides.liftMotor.setPositionPIDFCoefficients(P_Outake);

        this.intakeMechanism.update();
        this.outtakeSlides.update();

        if (target == ArmTarget.Intake) {
            if (this.intakeMechanism.currentMode == SelectiveIntakeMotor.CurrentMode.Taken) {
                target = target.next;
            }
        } else if (target == ArmTarget.OuttakeRetract) {
            if (outakeEnd.isPressed()) {
                target = target.next;
            }
        } else if (target == ArmTarget.OuttakeExtend) {
            if (Math.abs(outtakeSlides.currentPosition() - outtakeSlides.targetDistance) < 0.5) {
                target = target.next;
            }
        } else if (target == ArmTarget.InternalTransferPt1) {
            double cradleDistance = transferCradle.getDistance(DistanceUnit.MM);
            Logging.LOG("Cradle Distance", cradleDistance);
            if (cradleDistance < 20) {
                target = target.next;
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
                case Intake:
                    this.intakeMechanism.startSeeking();

                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(3);
                    this.wrist.changeTo(3);
                    break;

                case Retracted:
                case OuttakeRetract:
                    this.intakeMechanism.stopSeeking();
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(0);
                    this.wrist.changeTo(0);
                    break;


                case InternalTransferPt1:
                    this.intakeMechanism.stopSeeking();
                    this.intakeMechanism.overrideMotor(TransferSpeed);
                    this.intakeMechanism.currentMode = SelectiveIntakeMotor.CurrentMode.Idle;

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(3);
                    this.wrist.changeTo(3);
                    break;

                case InternalTransferPt2:
                    this.intakeMechanism.overrideMotor(0);
                    this.intakeMechanism.currentMode = SelectiveIntakeMotor.CurrentMode.Idle;

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(0);
                    this.wrist.changeTo(0);
                    break;

                case InternalWait:
                    this.intakeMechanism.overrideMotor(0);
                    this.intakeMechanism.currentMode = SelectiveIntakeMotor.CurrentMode.Idle;

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(true);
                    this.elbow.changeTo(0);
                    this.wrist.changeTo(0);
                    break;


                case OuttakePartialExtend:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(outtakeDistance);
                    this.outtakeGrabber.changeTo(true);
                    this.elbow.changeTo(0);
                    this.wrist.changeTo(0);
                    break;

                case OuttakeExtend:
                case OuttakeReady:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(outtakeDistance);
                    this.outtakeGrabber.changeTo(true);
                    this.elbow.changeTo(1);
                    this.wrist.changeTo(1);
                    break;

                case Outtake:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(outtakeDistance);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(1);
                    this.wrist.changeTo(1);
                    break;

                case SpecimenSmallLift:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(3);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(0);
                    this.wrist.changeTo(0);
                    break;

                case SpecimenLiftFold:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(3);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(2);
                    this.wrist.changeTo(2);
                    break;

                case SpecimenWait:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(false);
                    this.elbow.changeTo(2);
                    this.wrist.changeTo(2);
                    break;

                case SpecimenReady:
                    this.intakeMechanism.overrideMotor(0);

                    this.outtakeSlides.setDistance(0.1);
                    this.outtakeGrabber.changeTo(true);
                    this.elbow.changeTo(2);
                    this.wrist.changeTo(2);
                    break;
            }
        }
    }

    public void reset() {
        this.target = this.target.safeBack;
    }

    public void resetLifts() {
        this.outtakeSlides.reset(outakeEnd::isPressed);
    }
}
