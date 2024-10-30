package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.galahlib.Button;

public class ContinuousServoToggle {
    final Double off;
    final Double on;
    private final Button button;
    private final CRServo servo;


    public ContinuousServoToggle(HardwareMap hardwareMap, String name, Double off, Double on) {
        this.button = new Button(false, this::onChange);
        this.servo = hardwareMap.get(CRServo.class, name);
        servo.setPower(off);
        this.off = off;
        this.on = on;
    }

    public void onChange(Boolean new_val) {
        servo.setPower(new_val ? off : on);
    }

    public void update(boolean new_val) {
        button.update(new_val);
    }

    public void changeTo(Boolean new_val) {
        onChange(new_val);
        button.val = new_val;
    }

    public boolean get() {
        return button.val;
    }
}
