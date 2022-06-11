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

import java.util.Objects;

public class LinearActuator {
    private DcMotorEx motor;
    Utility utility = new Utility();
    String name;
    Telemetry telemetry;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double TICKS_PER_REV;
    public double TICKS_PER_DEGREE;
    public double TICKS_PER_INCH;
    public double INCHES_PER_ROTATION;
    public double EFFECTIVE_RADIUS;
    private double maxDistance;
    private double minDistance;
    private double targetDistance;
    private final MotorConstants motorConstants = new MotorConstants();

    // Pid stuff
    public PIDFController controller;
    // Must call setCoefficients to use this
    public void setCoefficients(PIDCoefficients coefficients) {
        controller = new PIDFController(coefficients);
    }


    // Constructors
    public LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio, double inchesPerRotation) {
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
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double inchesPerRotation) {
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
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    // Methods
    // Setting power on a linear mech is probably a dangerous thing to do most of the time
    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }
    public void stop() {
        motor.setPower(0);
    }
    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void reverse(){motor.setDirection(DcMotorSimple.Direction.REVERSE);}

    // Position things
    public void setLimits(double min, double max){
        minDistance = min;
        maxDistance = max;
    }

    public double getMaxDistance() {
        return maxDistance;
    }

    public  double getMinDistance() {
        return minDistance;
    }

    public void runToDistance(double distance) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetDistance = utility.clipValue(minDistance, maxDistance, distance);
        controller.setTargetPosition(targetDistance);
        motor.setPower(controller.update(getCurrentDistance()));
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public double getCurrentDistance() {
        return motor.getCurrentPosition() / TICKS_PER_INCH;
    }


    // Miscellaneous methods
    public void connectTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(100);
    }

    public void displayDebugInfo(){
        telemetry.addData("Current distance", getCurrentDistance());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Min distance", minDistance);
        telemetry.addData("Max distance", maxDistance);
        telemetry.addData("Controller output", controller.update(getCurrentDistance()));
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Current", getCurrent());
        telemetry.addData("Ticks per degree", TICKS_PER_DEGREE);
        telemetry.addData("Tick per rev", TICKS_PER_REV);
        telemetry.addData("Ticks per inch", TICKS_PER_INCH);
        telemetry.update();
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    public double getVelocityInRotations() {
       return motor.getVelocity() / TICKS_PER_REV;
    }
    public double getVelocityInDegrees() {
        return motor.getVelocity() / TICKS_PER_DEGREE;
    }
}
