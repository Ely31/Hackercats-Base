package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoEx {
    private ServoImplEx servo;
    private double travel;

    // Constructors
    public ServoEx(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(ServoImplEx.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Extend range to get the most angle out of it
        travel = 180;
    }
    public ServoEx(HardwareMap hardwareMap, String name, double travel) {
        servo = hardwareMap.get(ServoImplEx.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Extend range to get the most angle out of it
        this.travel = travel;
    }

    // Methods
    public void setRange(double min, double max) {
        servo.setPwmRange(new PwmControl.PwmRange(min, max));
    }
    public PwmControl.PwmRange getRange() {
        return servo.getPwmRange();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
    public void setAngle(double angle) {
        servo.setPosition(angle / travel);
    }
    public double getAngle() {
        return servo.getPosition() * travel;
    }

    public void setDirection(boolean isInverted) {
        servo.setDirection(isInverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
    public Servo.Direction getDirection() {
        return servo.getDirection();
    }
}
