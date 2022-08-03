package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.actuators.PIDActuator;

@Config
@TeleOp(group = "test")
public class ActuatorTestPID extends LinearOpMode {
    // Pre init
    PIDActuator test;
    public static String name = "carousel";
    @Override
    public void runOpMode(){
        // Init
        test = new PIDActuator(hardwareMap, name, 3.7);
        test.setPositionCoefficients(new PIDCoefficients (0.004, 0.0001, 0.0001));
        test.setLimits(0,360);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            if (gamepad1.a) {
                test.runToAngle(180);
            }
            else test.runToAngle(0);

            test.updatePositionControl();

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("current angle", test.getCurrentAngle());
            telemetry.addData("target angle", test.getTargetAngle());
            telemetry.addData("electical current", test.getCurrent());
            telemetry.addData("power", test.getPower());
            telemetry.addData("controller error", test.positionController.getLastError());
            telemetry.addData("controller target", test.positionController.getTargetPosition());
            telemetry.addData("controller update", test.positionController.update(test.getCurrentAngle()));
            telemetry.update();

        }
    }
}