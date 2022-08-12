package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;
    String name;

    double distance;

    public DistanceSensor(HardwareMap hardwareMap, String name){
        this.name = name;
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, name);
    }

    public double getDistance(){return distance;}

    // You MUST call update every loop
    public void update(){
        // The -1 number was determined from testing, the sensor always reported about a cm over the actual distance
        distance = distanceSensor.getDistance(DistanceUnit.CM) -1;
    }

    public void displayDebugInfo(Telemetry telemetry){
        telemetry.addData("distance", getDistance());
    }
}
