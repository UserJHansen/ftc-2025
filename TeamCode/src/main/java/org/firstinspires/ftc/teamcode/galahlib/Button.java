package org.firstinspires.ftc.teamcode.galahlib;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
public class Button {
    public boolean val;
    public ButtonPressed whenPressed;
    private boolean isPressed = false;

    public static long debounceIntervalMS = 50;

    public interface ButtonPressed {
        void call(boolean param);
    }

    public Button(boolean default_val, ButtonPressed whenPressed) {
        this.val = default_val;
        this.whenPressed = whenPressed;
    }

    public Button(boolean default_val) {
        this.val = default_val;
    }

    public Button() {
        this.val = false;
    }

    private Deadline timeout = new Deadline(debounceIntervalMS, TimeUnit.MILLISECONDS);
    public Boolean update(boolean new_val) {
        if (new_val && !isPressed && timeout.hasExpired()) {
            val = !val;
            if (whenPressed != null)
                whenPressed.call(val);
            timeout = new Deadline(debounceIntervalMS, TimeUnit.MILLISECONDS);
            return true;
        }
        isPressed = new_val;
        return false;
    }
}
