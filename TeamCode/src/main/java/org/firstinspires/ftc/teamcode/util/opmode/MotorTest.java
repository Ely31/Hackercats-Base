package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GamepadUtilOld;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
@TeleOp(group="test")
public class MotorTest extends LinearOpMode {
    // Pre-init
    DcMotor testMotor;
    Utility utility = new Utility();
    GamepadUtilOld toggleMode = new GamepadUtilOld();
    GamepadUtilOld toggleDirection = new GamepadUtilOld();
    public static String motorName = "test";
    public static int maxPower = 1;
    public static double gearboxRatio = 19.203;
    public static double externalGearRatio = 1.0/1.0;
    double ticksPerRev = gearboxRatio * 28 /externalGearRatio; // This equals the ticks per rev on the final output of the system
    double ticksPerDegree = (ticksPerRev / 360.0);

    public static double minAngle = 0;
    public static double maxAngle = 90;
    public static double angleOne = 30;
    public static double angleTwo = 60;

    boolean mode = false; // False means we are in power mode, true means we are in position mode

    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Graph stuff on dashboard

        testMotor = hardwareMap.dcMotor.get(motorName);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setTargetPosition(0);

        telemetry.setMsTransmissionInterval(100);
        telemetry.update();
        telemetry.addLine("WARNING: putting in the wrong numbers could be dangerous, depending on the mechanism");
        telemetry.addLine("Use FTC Dash to configure the motor");
        telemetry.update();

        waitForStart();
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop

            if (toggleMode.isFirstPressed(gamepad1.dpad_left)) {
                mode = !mode;
            }
            if (!mode) {
                // Power mode
                testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                testMotor.setPower((-gamepad1.left_stick_y*maxPower) + (-gamepad1.right_stick_y*maxPower*0.5)); // Control with sticks

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
                if (gamepad1.a) runToAngle(minAngle);
                if (gamepad1.b) runToAngle(maxAngle);
                if (gamepad1.x) runToAngle(angleOne);
                if (gamepad1.y) runToAngle(angleTwo);
            }
            // Telemtry
            telemetry.addData("Mode", mode ? "Position" : "Power");
            telemetry.addData("Ticks per rev", ticksPerRev);
            telemetry.addData("Ticks per degree", ticksPerDegree);
            telemetry.addData("Direction", testMotor.getDirection());
            telemetry.addData("Power", testMotor.getPower());
            telemetry.addData("Target positon in ticks", testMotor.getTargetPosition());
            telemetry.addData("Position in ticks", testMotor.getCurrentPosition());
            telemetry.addData("Position in degrees", testMotor.getCurrentPosition() / ticksPerDegree);
            telemetry.update();
        }
    }
    void runToAngle(double angle) {
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        testMotor.setPower(maxPower);
        testMotor.setTargetPosition((int) (utility.clipValue(minAngle, maxAngle, angle) * ticksPerDegree));
    }
}
