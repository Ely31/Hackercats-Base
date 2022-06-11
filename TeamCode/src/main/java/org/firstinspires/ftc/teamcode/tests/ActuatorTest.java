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
    @Override
    public void runOpMode(){
        // Init
        test = new Actuator(hardwareMap, "carousel", 3.7);
        test.setLimits(0,360);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) {test.runToAngle(180);}
            else test.runToAngle(0);

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("current angle", test.getCurrentAngle());
            telemetry.addData("target angle", test.getTargetAngle());
            telemetry.addData("electical current", test.getCurrent());
            telemetry.update();

        }
    }
}