package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

public class ServoMultiState {
    final double[] states;
    private final Button button;
    private final Servo servo;
    public int state = 0;


    public ServoMultiState(HardwareMap hardwareMap, String name, double[] states) {
        this.button = new Button(false, this::onChange);
        this.servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(states[0]);
        this.states = states;
    }

    public void onChange(Boolean new_val) {
        state = (state + 1) % states.length;
        servo.setPosition(states[state]);
    }

    public void update(boolean new_val) {
        button.update(new_val);
    }

    public void changeTo(int new_val) {
        state = new_val % states.length;
        servo.setPosition(states[state]);
    }

    public void changeTo(boolean new_val) {
        servo.setPosition(states[new_val ? 1 : 0]);
    }

    public int get() {
        return this.state;
    }
}
