package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.actuators.Actuator;

@Config
@TeleOp(group = "test")
public class ActuatorTest extends LinearOpMode {
    // Pre init
    Actuator test;
    String name;
    @Override
    public void runOpMode(){
        // Init
        test = new Actuator(hardwareMap, name, 3.7);
        test.setLimits(0,360);
        test.connectTelemetry(telemetry);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) {test.runToAngle(180);}
            else test.runToAngle(0);

            test.displayDebugInfo();
            telemetry.update();
        }
    }
}