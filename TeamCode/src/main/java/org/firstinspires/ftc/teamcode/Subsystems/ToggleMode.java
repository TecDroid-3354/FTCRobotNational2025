package org.firstinspires.ftc.teamcode.Subsystems;

public class ToggleMode {
    boolean btnPressed = false;

    public boolean toggleBtn(boolean btn) {
        if (btn && !btnPressed){
            btnPressed = true;
            return true;
        }

        if (!btn) {
            btnPressed = false;
        }

        return false;
    }
}
