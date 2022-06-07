package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.motor.Actuator;
import org.firstinspires.ftc.teamcode.hardware.motor.PIDActuator;

@Config
@TeleOp(group = "test")
public class ActuatorTestPID extends LinearOpMode {
    // Pre init
    PIDActuator test;
    @Override
    public void runOpMode(){
        // Init
        test = new PIDActuator(hardwareMap, "carousel", 3.7);
        test.setPIDCoefficients(0.004, 0.0001, 0.0001);
        test.setLimits(0,360);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) {test.runToAngle(180);}
            else test.runToAngle(0);

            test.update();

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("current angle", test.getCurrentAngle());
            telemetry.addData("target angle", test.getTargetAngle());
            telemetry.addData("electical current", test.getCurrent());
            telemetry.update();

        }
    }
}