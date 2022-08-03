package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.actuators.PIDActuator;

@Config
@TeleOp(group="test")
public class MotorTestVelo extends LinearOpMode {
    // Pre-init
    PIDActuator motor;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(10,2,10,0);
    public static double multiplier = 5;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = new PIDActuator(hardwareMap, "carousel", 3.7, 1.0);
        motor.connectTelemetry(telemetry);
        telemetry.setMsTransmissionInterval(100);
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            motor.setVelocityCoefficients(coefficients);
            motor.setVelocity(gamepad1.left_stick_y * multiplier);
            motor.displayVelocityDebugInfo();
            telemetry.update();
        }
    }
}
