package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;

@TeleOp
public class BaseTeleOp extends LinearOpMode {
    // Pre init
    TeleMecDrive drive;
    @Override
    public void runOpMode(){
        // Init
        drive = new TeleMecDrive(hardwareMap, 0.5);
        waitForStart();
        while (opModeIsActive()){
            // Teleop code
            drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);
            if (gamepad1.back) drive.resetHeading();
        }
    }
}