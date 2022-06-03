package org.firstinspires.ftc.teamcode.util;

public class TruePress {

    private boolean prevInput = false;

    // To make something only run once a button is pressed for the firt time
    public boolean isFirstPressed(boolean input){
        boolean output;
        output = !prevInput && input;
        prevInput = input;
        return output;
    }
}
