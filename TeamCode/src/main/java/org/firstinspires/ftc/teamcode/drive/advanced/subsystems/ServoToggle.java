package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

public class ServoToggle {
    final Double off;
    final Double on;
    private final Button button;
    private final Servo servo;


    public ServoToggle(HardwareMap hardwareMap, String name, Double off, Double on) {
        this.button = new Button(false, this::onChange);
        this.servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(off);
        this.off = off;
        this.on = on;
    }

    public void onChange(Boolean new_val) {
        servo.setPosition(new_val ? on : off);
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
