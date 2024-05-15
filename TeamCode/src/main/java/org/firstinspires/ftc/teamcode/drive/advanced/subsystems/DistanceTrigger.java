package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.galahlib.ButtonPressed;

public class DistanceTrigger {
    public boolean val;
    public ButtonPressed onChange;
    public ColorRangeSensor sensor;
    public double triggerDistance;

    public DistanceTrigger(double triggerDistance, ColorRangeSensor sensor, ButtonPressed onChange) {
        this.onChange = onChange;
        this.sensor = sensor;
        this.triggerDistance = triggerDistance;
    }

    public DistanceTrigger(double triggerDistance, ColorRangeSensor sensor) {
        this.sensor = sensor;
        this.triggerDistance = triggerDistance;
    }

    public void update() {
        boolean oldVal = val;
        val = sensor.getDistance(DistanceUnit.CM) < triggerDistance;

        if (oldVal != val && onChange != null) {
            onChange.call(val);
        }
    }
}
