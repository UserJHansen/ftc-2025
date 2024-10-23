package org.firstinspires.ftc.teamcode.galahlib;

public class Button {
    public boolean val;
    public ButtonPressed whenPressed;
    private boolean isPressed = false;

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

    public Boolean update(boolean new_val) {
        if (new_val && !isPressed) {
            val = !val;
            if (whenPressed != null)
                whenPressed.call(val);
            return true;
        }
        isPressed = new_val;
        return false;
    }
}
