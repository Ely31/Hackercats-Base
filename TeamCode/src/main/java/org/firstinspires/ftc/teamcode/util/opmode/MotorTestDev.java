package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.TruePress;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
@TeleOp(group="test")
public class MotorTestDev extends LinearOpMode {
    // Pre-init
    DcMotor testMotor;
    Utility utility = new Utility();
    TruePress toggleMode = new TruePress();
    TruePress toggleDirection = new TruePress();
    public static String motorName = "test";
    public static int maxPower = 1;
    public static double gearboxRatio = 19.203;
    public static double externalGearRatio = 1.0/1;
    double ticksPerRev = gearboxRatio * 28 /externalGearRatio; // This equals the ticks per rev on the final output of the system
    int TicksPerDegree = (int) (ticksPerRev / 360.0);

    public static double minAngle = 0;
    public static double maxAngle = 90;
    public static double angleOne = 30;
    public static double angleTwo = 60;

    boolean mode = false; // False means we are in power mode, true means we are in position mode

    @Override
    public void runOpMode() {
        // Init
        testMotor = hardwareMap.dcMotor.get(motorName);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setTargetPosition(0);

        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("WARNING: putting in the wrong numbers could be dangerous, depending on the mechanism");
        telemetry.addLine("Use FTC Dash to configure the motor");
        telemetry.update();
        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            if (toggleMode.isFirstPressed(gamepad1.a)) {
                mode = !mode;
            }
            if (!mode) {
                // Power mode
                testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                testMotor.setPower(-gamepad1.left_stick_y); // Control with left stick

                // Switch directions with back button
                if (toggleDirection.isFirstPressed(gamepad1.back)) {
                    if (testMotor.getDirection() == DcMotor.Direction.FORWARD) {
                        testMotor.setDirection(DcMotor.Direction.REVERSE);
                    } else {
                        testMotor.setDirection(DcMotor.Direction.FORWARD);
                    }
                }
            }
            else {
                // Position mode
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad1.a) runToAngle(minAngle);
                if (gamepad1.x) runToAngle(maxAngle);
                if (gamepad1.y) runToAngle(angleOne);
                if (gamepad1.b) runToAngle(angleTwo);
            }
            // Telemtry
            telemetry.addData("Mode", mode ? "Position" : "Power");
            telemetry.addData("ticks per rev", ticksPerRev);
            telemetry.addData("direction", testMotor.getDirection());
            telemetry.addData("Power", testMotor.getPower());
            telemetry.addData("Position in ticks", testMotor.getCurrentPosition());
            telemetry.addData("Position in degrees", testMotor.getCurrentPosition() / TicksPerDegree);
            telemetry.update();
        }
    }
    void runToAngle(double angle) {
        testMotor.setTargetPosition((int) (utility.clipValue(minAngle, maxAngle, angle) * TicksPerDegree));
    }
}
