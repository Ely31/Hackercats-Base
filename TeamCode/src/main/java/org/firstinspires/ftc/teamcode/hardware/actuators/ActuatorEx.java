package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ActuatorEx extends Actuator{

    // For linear motion with a rack and pinion or spool and string, in centimeters
    private double EFFECTIVE_DIAMETER;
    private double EFFECTIVE_CIRCUMFERENCE;
    private double TICKS_PER_CM;

    // Pid stuff
    public PIDFController positionController;
    // Must call setCoefficients to use any pid features
    public void setPositionCoefficients(PIDCoefficients coefficients) {
        positionController = new PIDFController(coefficients);
    }
    public void setVelocityCoefficients(PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    // Constructors
    public ActuatorEx(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio) {
        super(hardwareMap, name, gearboxRatio, externalGearRatio);
    }
    public ActuatorEx(HardwareMap hardwareMap, String name, double gearboxRatio) {
        super(hardwareMap, name, gearboxRatio);
    }

    // PID position things
    public void setAnglePID(double angle) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = utility.clipValue(minAngle, maxAngle, angle);
        positionController.setTargetPosition(target);
    }

    // You MUST call update EVERY loop for the pid controller to work
    public void update(){motor.setPower(positionController.update(getCurrentAngle()));}
    
    // Linear motion things
    public void setDiameter(double diameter){
        // Convert diameter in cm into ticks per cm of tangential movement
        EFFECTIVE_DIAMETER = diameter;
        EFFECTIVE_CIRCUMFERENCE = EFFECTIVE_DIAMETER * Math.PI;
        TICKS_PER_CM = TICKS_PER_REV / EFFECTIVE_CIRCUMFERENCE;
    }
}
