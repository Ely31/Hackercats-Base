package org.firstinspires.ftc.teamcode.util;

public class GamepadUtilOld {
    // You should be able to use this on triggers as well if you make the input (trigger > 0.1).
    private boolean prevInput = false;

    // To make something only run once a button is pressed for the firt time (aka rising edge)
    public boolean isFirstPressed(boolean input){
        boolean output;
        output = !prevInput && input;
        prevInput = input;
        return output;
    }

    // Falling edge
    public boolean isFirstReleased(boolean input){
        boolean output;
        output = prevInput && !input;
        prevInput = input;
        return output;
    }
}
