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
        //test.setLimits(0,360);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) {test.runToAngleRTP(180);}
            else test.runToAngleRTP(0);

            test.displayDebugInfo(telemetry);
            telemetry.update();
        }
    }
}