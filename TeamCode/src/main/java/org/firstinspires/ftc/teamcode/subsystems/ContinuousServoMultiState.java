package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.galahlib.Button;

public class ContinuousServoMultiState {
    final double[] states;
    private final Button button;
    private final CRServo servo;
    public int state = 0;


    public ContinuousServoMultiState(HardwareMap hardwareMap, String name, double[] states) {
        this.button = new Button(false, this::onChange);
        this.servo = hardwareMap.get(CRServo.class, name);
        servo.setPower(states[0]);
        this.states = states;
    }

    public void onChange(Boolean new_val) {
        state = (state + 1) % states.length;
        servo.setPower(states[state]);
    }

    public void update(boolean new_val) {
        button.update(new_val);
    }

    public void changeTo(int new_val) {
        state = new_val % states.length;
        servo.setPower(states[state]);
    }

    public void changeTo(boolean new_val) {
        servo.setPower(states[new_val ? 1 : 0]);
    }

    public int get() {
        return this.state;
    }

}
