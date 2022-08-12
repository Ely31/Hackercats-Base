package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ImuSimple {
    BNO055IMU imu;
    String name;

    double rawAngle;
    double angle;
    double offset;

    public ImuSimple(HardwareMap hardwareMap, String name){
        this.name = name;
        imu = hardwareMap.get(BNO055IMU.class, name);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
    public double getAngle(){return angle;}
    public void zeroHeading(){offset = -rawAngle;}

    public void update(){
        rawAngle = imu.getAngularOrientation().firstAngle;
        angle = rawAngle + offset;
    }
}
