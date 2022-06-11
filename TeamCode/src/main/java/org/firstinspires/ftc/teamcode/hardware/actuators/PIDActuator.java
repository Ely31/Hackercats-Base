package org.firstinspires.ftc.teamcode.hardware.actuators;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

public class PIDActuator {
    private DcMotorEx motor;
    Utility utility = new Utility();
    String name;
    Telemetry telemetry;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double TICKS_PER_REV;
    public double TICKS_PER_DEGREE;
    private double maxAngle;
    private double minAngle;
    private double targetAngle;
    private final MotorConstants motorConstants = new MotorConstants();

    // Pid stuff
    public PIDFController controller;
    // Must call setCoefficients to use this
    public void setCoefficients(PIDCoefficients coefficients) {
        controller = new PIDFController(coefficients);
    }

    // Constructors
    public PIDActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio) {
        this.name = name;
        GEARBOX_RATIO = gearboxRatio;
        EXTERNAL_GEAR_RATIO = externalGearRatio;

        if (GEARBOX_RATIO == 1) TICKS_PER_REV = motorConstants.BARE * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 3.7) TICKS_PER_REV = motorConstants.TICKS_37 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 5.2) TICKS_PER_REV = motorConstants.TICKS_52 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 13.7) TICKS_PER_REV = motorConstants.TICKS_137 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 19.2) TICKS_PER_REV = motorConstants.TICKS_192 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 26.9) TICKS_PER_REV = motorConstants.TICKS_269 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 50.9) TICKS_PER_REV = motorConstants.TICKS_509 * EXTERNAL_GEAR_RATIO;
        if (TICKS_PER_REV == 0.0) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public PIDActuator(HardwareMap hardwareMap, String name, double gearboxRatio) {
        this.name = name;
        GEARBOX_RATIO = gearboxRatio;
        EXTERNAL_GEAR_RATIO = 1;

        if (GEARBOX_RATIO == 1) TICKS_PER_REV = motorConstants.BARE * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 3.7) TICKS_PER_REV = motorConstants.TICKS_37 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 5.2) TICKS_PER_REV = motorConstants.TICKS_52 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 13.7) TICKS_PER_REV = motorConstants.TICKS_137 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 19.2) TICKS_PER_REV = motorConstants.TICKS_192 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 26.9) TICKS_PER_REV = motorConstants.TICKS_269 * EXTERNAL_GEAR_RATIO;
        if (GEARBOX_RATIO == 50.9) TICKS_PER_REV = motorConstants.TICKS_509 * EXTERNAL_GEAR_RATIO;
        if (TICKS_PER_REV == 0.0) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // Methods
    public void stop() {
        motor.setPower(0);
    }
    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void reverse() {motor.setDirection(DcMotorSimple.Direction.REVERSE);}

    // Position things
    public void setLimits(double min, double max){
        minAngle = min;
        maxAngle = max;
    }
    public double getMaxAngle() {
        return maxAngle;
    }
    public  double getMinAngle() {
        return minAngle;
    }

    public void runToAngle(double angle) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = utility.clipValue(minAngle, maxAngle, angle);
        controller.setTargetPosition( targetAngle);
        motor.setPower(controller.update(getCurrentAngle()));
    }
    // MAKE SURE TO CALL UPDATE EVERY LOOP!
    public void update(){
        controller.update(getCurrentAngle());
    }

    public double getTargetAngle() {
        return targetAngle;
    }
    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    // Miscellaneous methods
    public void connectTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(100);
    }
    public void displayDebugInfo() {
        telemetry.addData("Current angle", getCurrentAngle());
        telemetry.addData("Target angle", getTargetAngle());
        telemetry.addData("Controller output", controller.update(getCurrentAngle()));
        telemetry.addData("Min", minAngle);
        telemetry.addData("Max", maxAngle);
        telemetry.addData("Runmode", motor.getMode());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Current", getCurrent());
        telemetry.addData("Ticks per degree", TICKS_PER_DEGREE);
        telemetry.addData("Tick per rev", TICKS_PER_REV);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    public double getPower() {return motor.getPower();}

    public double getVelocityInRotations() {
        return motor.getVelocity() / TICKS_PER_REV;
    }
    public double getVelocityInDegrees() {
        return motor.getVelocity() / TICKS_PER_DEGREE;
    }
}
