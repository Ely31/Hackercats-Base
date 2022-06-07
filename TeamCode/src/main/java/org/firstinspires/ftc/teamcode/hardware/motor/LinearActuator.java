package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.Objects;

public class LinearActuator {
    private DcMotorEx motor;
    Utility utility = new Utility();
    HardwareMap hardwareMap;
    String name;
    public double GEARBOX_RATIO;
    public double EXTERNAL_GEAR_RATIO = 1.0 / 1.0;
    public double TICKS_PER_REV;
    public double TICKS_PER_DEGREE;
    public double TICKS_PER_INCH;
    public double INCHES_PER_ROTATION;
    public double EFFECTIVE_RADIUS;
    private double maxDistance;
    private double minDistance;
    private final MotorConstants motorConstants = new MotorConstants();

    private double targetAngle;


    // Constructors
    LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio, double inchesPerRotation) {
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
        if (Objects.isNull(TICKS_PER_REV)) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double inchesPerRotation) {
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
        if (Objects.isNull(TICKS_PER_REV)) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double externalGearRatio, double inchesPerRotation, boolean reversed) {
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
        if (Objects.isNull(TICKS_PER_REV)) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    LinearActuator(HardwareMap hardwareMap, String name, double gearboxRatio, double inchesPerRotation, boolean reversed) {
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
        if (Objects.isNull(TICKS_PER_REV)) {
            throw new IllegalArgumentException("Invalid gearbox ratio");
        }
        INCHES_PER_ROTATION = inchesPerRotation;
        TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_ROTATION;
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    // Methods
    // Power and primative things
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

    public void runToDistance(double angle) { // Make sure to use .setLimits before using this
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetAngle = utility.clipValue(minDistance, maxDistance, angle);
        motor.setTargetPosition((int) (targetAngle * TICKS_PER_DEGREE));
    }

    public double getTargetDistance() {
        return targetAngle;
    }

    public double getCurrentDistance() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }


    // Miscellaneous methods
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
