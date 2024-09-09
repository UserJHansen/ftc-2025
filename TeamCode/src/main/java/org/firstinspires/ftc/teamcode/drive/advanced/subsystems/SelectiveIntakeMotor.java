package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

public class SelectiveIntakeMotor {
    final double speed;

    private final Button motorActive;
    private final DcMotor motor;
    private final ColorRangeSensor sensor;

    public boolean active = false;

    public SelectiveIntakeMotor(HardwareMap hardwareMap, String motorName, String sensorName, double speed) {
        this.motorActive = new Button(false, this::onChange);
        this.speed = speed;

        this.motor = hardwareMap.get(DcMotor.class, motorName);
        this.sensor = hardwareMap.get(ColorRangeSensor.class, sensorName);
    }

    public void onChange(Boolean new_val) {
        this.motor.setPower(new_val ? speed : 0);
    }

    public void update() {
        this.motorActive.update(get());
    }

    public void overrideMotor(double motorSpeed) {
        if (!this.active) this.motor.setPower(motorSpeed);
    }

    public SampleType getSampleType() {
        NormalizedRGBA color = this.sensor.getNormalizedColors();

        if (color.red > 0.8 && color.green < 0.3 && color.blue < 0.3) {
            return SampleType.Red;
        } else if (color.red < 0.3 && color.green < 0.3 && color.blue > 0.8) {
            return SampleType.Blue;
        } else if (color.red > 0.8 && color.green > 0.45 && color.blue < 0.3) {
            return SampleType.Shared;
        } else {
            return SampleType.Unknown;
        }
    }

    public boolean hasSample() {
        SampleType sample = this.getSampleType();
        return this.sensor.getDistance(DistanceUnit.MM) < 20 && (sample == SampleType.Shared || sample == (PoseStorage.isRedAlliance ? SampleType.Red : SampleType.Blue));
    }

    public boolean get() {
        return this.active && this.hasSample();
    }
}
