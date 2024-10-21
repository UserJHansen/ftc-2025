package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

import java.util.concurrent.TimeUnit;

@Config
public class SelectiveIntakeMotor {
    private final DcMotorEx motor;
    private final ColorRangeSensor sensor;
    private final Rev2mDistanceSensor confirmationSensor;

    private final Lift intakeSlides;
    private final ServoToggle intakeFlip;

    public CurrentMode currentMode = CurrentMode.Idle;

    public enum CurrentMode {
        Seeking,
        Taken,
        Idle
    }

    public static double speed = -0.53;
    public static double minExtension = 5;
    public static double maxExtension = 11;
    public static double extendSpeed = 2; // Inches per second

    public static double P_INTAKE = 6;

    private double slideTarget = 0;
    private InternalState currentState = InternalState.Retracted;
    public enum InternalState {
        Retracted,
        Extending,
        Seeking,
        Retracting
    }

    public SelectiveIntakeMotor(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "intake");
        this.sensor = hardwareMap.get(ColorRangeSensor.class,  "intakeColour");
        this.confirmationSensor = hardwareMap.get(Rev2mDistanceSensor.class, "intakeDistance");

        intakeSlides = new Lift(hardwareMap.get(DcMotorEx.class, "intakeSlides"), DcMotor.Direction.REVERSE, new PIDFCoefficients(P_INTAKE, 0, 0, 0));
        intakeFlip = new ServoToggle(hardwareMap, "intakeFlip", 0.05, 0.35); // up state is off, Down state is on

        DigitalChannel leftEndpoint = hardwareMap.get(DigitalChannel.class, "leftIntakeEndpoint");
        DigitalChannel rightEndpoint = hardwareMap.get(DigitalChannel.class, "rightIntakeEndpoint");

        intakeSlides.reset(() -> !leftEndpoint.getState() || !rightEndpoint.getState()); // Wait for either to get triggered

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startSeeking() {
        if (currentMode == CurrentMode.Idle) {
            currentMode = CurrentMode.Seeking;
            currentState = InternalState.Extending;
            slideTarget = minExtension;
            intakeSlides.setDistance(slideTarget);
        }
    }

    public void stopSeeking() {
        if (currentMode == CurrentMode.Seeking) {
            currentMode = CurrentMode.Idle;
            currentState = InternalState.Retracting;
            slideTarget = 0;
            intakeSlides.setDistance(slideTarget);
        }
    }

    ElapsedTime timer = new ElapsedTime();
    private SampleType previousSample = SampleType.Unknown;
    private Deadline captureTimeout;
    private Deadline foldTimeout;
    public void update() {
        double elapsedTime = timer.milliseconds();
        Logging.LOG("CPS", 1000 / elapsedTime);
        Logging.LOG("(Intake) Current Mode", currentMode);
        Logging.LOG("(Intake) Capture timeout", captureTimeout);
        Logging.LOG("(Intake) Fold timeout", foldTimeout);
        Logging.LOG("(Intake) Intake internal state", currentState);
        Logging.LOG("(Intake) Current Position", intakeSlides.currentPosition());
        Logging.LOG("(Intake) Target Position", slideTarget);
        timer.reset();

        intakeSlides.liftMotor.setPositionPIDFCoefficients(P_INTAKE);
        if (currentState == InternalState.Retracting && intakeSlides.currentPosition() < minExtension) {
            this.currentState = InternalState.Retracted;
            this.currentMode = CurrentMode.Taken;
        } else if (currentState == InternalState.Extending && intakeSlides.currentPosition() > minExtension - 0.1) {
            this.currentState = InternalState.Seeking;
            this.intakeFlip.changeTo(true);
        }

        if (this.currentMode == CurrentMode.Seeking) {
            if (this.captureTimeout != null && this.captureTimeout.hasExpired()) {
                captureTimeout = null;
                foldTimeout = new Deadline(500, TimeUnit.MILLISECONDS);
                this.intakeFlip.changeTo(false);
                slideTarget = 0;
                this.intakeSlides.setDistance(0);
                setPower(0.1);
            } else if (foldTimeout != null) {
                if (this.foldTimeout.hasExpired()) {
                    foldTimeout = null;
                    this.currentState = InternalState.Retracting;
                    slideTarget = 0;
                    this.intakeSlides.setDistance(0);
                }
            } else {
                if (currentState == InternalState.Seeking) {
                    SampleType colourSample = this.getSampleType();
                    double frontDistance = this.sensor.getDistance(DistanceUnit.MM);
                    double rearDistance = this.confirmationSensor.getDistance(DistanceUnit.MM);

                    slideTarget = Math.min(slideTarget + (extendSpeed * elapsedTime / 1000), maxExtension);
                    intakeSlides.setDistance(slideTarget);

                    Logging.DEBUG("(Intake) Current Sample", colourSample);
                    Logging.DEBUG("(Intake) Motor Power", this.motor.getPower());
                    Logging.DEBUG("(Intake) Motor Current", this.motor.getCurrent(CurrentUnit.AMPS));
                    Logging.DEBUG("(Intake) Distance Front", frontDistance);
                    Logging.DEBUG("(Intake) Distance End", rearDistance);

                    if (frontDistance < 25) previousSample = colourSample;
                    boolean goodSample = previousSample == SampleType.Shared || previousSample == (PoseStorage.isRedAlliance ? SampleType.Red : SampleType.Blue);

                    if (frontDistance < 25 && !goodSample) {
                        this.setPower(-1);
                        captureTimeout = null;
                    } else if (rearDistance < 60) {
                        this.setPower(0);
                        if (goodSample) {
                            this.setPower(0);
                            if (captureTimeout == null)
                                captureTimeout = new Deadline(500, TimeUnit.MILLISECONDS);
                        } else {
                            this.setPower(speed);
                            captureTimeout = null;
                        }
                    } else {
                        this.setPower(speed);
                        captureTimeout = null;
                    }
                }
            }
        }
    }

    private double lastSpeed;
    //    Cache the current speed
    void setPower(double newSpeed) {
        if (newSpeed != lastSpeed) {
            lastSpeed = newSpeed;
            this.motor.setPower(newSpeed);
        }
    }

    public void overrideMotor(double motorSpeed) {
        if (this.currentMode != CurrentMode.Seeking) this.setPower(motorSpeed);
    }

    public SampleType getSampleType() {
        NormalizedRGBA color = this.sensor.getNormalizedColors();

        float max = Math.max(Math.max(color.red, color.green), color.blue);

        color.red /= max;
        color.green /= max;
        color.blue /= max;

        Logging.DEBUG("Current colour", color.red + " " + color.green + " " + color.blue + " " + color.alpha);

        if (color.red == 1d) {
            return SampleType.Red;
        } else if (color.blue == 1d) {
            return SampleType.Blue;
        } else if (color.green == 1d) {
            return SampleType.Shared;
        } else {
            return SampleType.Unknown;
        }
    }
}
