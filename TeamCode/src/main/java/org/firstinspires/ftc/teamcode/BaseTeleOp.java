package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;

@TeleOp
public class BaseTeleOp extends LinearOpMode {
    // Pre init
    TeleMecDrive drive;
    @Override
    public void runOpMode(){
        // Init
        drive = new TeleMecDrive(hardwareMap, 0.4);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading();

            // Configure driving parameters with buttons
            if (gamepad1.a) drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad1.b) drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if (gamepad1.x) drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.y) drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("maxRPMFraction", drive.lfMaxRPMFraction);
            telemetry.addData("Runmode", drive.runMode);
            telemetry.addData("ZeroPowerBehavior", drive.zeroPowerBehavior);
            telemetry.update();
        }
    }
}