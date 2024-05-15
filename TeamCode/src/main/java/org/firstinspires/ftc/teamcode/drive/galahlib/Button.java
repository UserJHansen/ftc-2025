package org.firstinspires.ftc.teamcode.drive.galahlib;

public class Button {
    public boolean val;
    public ButtonPressed whenPressed;
    private boolean isPressed = false;

    public Button(boolean default_val, ButtonPressed whenPressed) {
        this.val = default_val;
        this.whenPressed = whenPressed;
    }

    public Button(boolean default_val) {
        this.val = default_val;
    }

    public void update(boolean new_val) {
        if (new_val && !isPressed) {
            val = !val;
            if (whenPressed != null)
                whenPressed.call(val);
        }
        isPressed = new_val;
    }
}
