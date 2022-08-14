package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.actuators.Actuator;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp(group = "test")
public class ActuatorTest extends LinearOpMode {
    // Pre init
    Actuator actuator;
    TimeUtil timeUtil = new TimeUtil();

    // Fsm setup stuff
    enum Mode{
        BASIC,
        ON_HUB_CONTROL,
        PID,
        LINEAR
    }
    Mode mode = Mode.BASIC;

    public static double min = 0;
    public static double max = 360;
    public static double gearboxRatio = 3.7;
    public static double externalRatio = 1.0/1.0;
    public static double diameter = 4;
    public static String name = "carousel";

    Gamepad prevGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();
    @Override
    public void runOpMode(){
        // Init
        PhotonCore.enable();
        actuator = new Actuator(hardwareMap, name, gearboxRatio);
        actuator.setDiameter(diameter);
        actuator.setLimits(min,max);

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();
        timeUtil.resetTimer();
        while (opModeIsActive()){
            // Teleop code
            try{
                prevGamepad.copy(currentGamepad);
                currentGamepad.copy(gamepad1);
            } catch (RobotCoreException ignored){
                // Idk why it wants a try catch statement
            }

            actuator.update();
            actuator.setLimits(min, max);
            timeUtil.update();

            // Fsm to switch modes
            switch (mode){
                case BASIC:
                    // Toggle direction with start
                    if (currentGamepad.start && !prevGamepad.start) {
                       if (actuator.getDirection() == DcMotorSimple.Direction.FORWARD) actuator.setDirection(DcMotorSimple.Direction.REVERSE);
                       else actuator.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    // Control power with sticks, right stick is half power
                    actuator.setPower((-currentGamepad.left_stick_y) + (-currentGamepad.right_stick_y *0.5));
                    gamepad1.setLedColor(1,0,0, -1);

                    if (sharePressed()) {
                        actuator.stop();
                        mode = Mode.ON_HUB_CONTROL;
                    }
                    break;

                case ON_HUB_CONTROL:


                    if (sharePressed()){
                        mode = Mode.PID;
                    }
                    break;

                case PID:


                    if (sharePressed()){
                        mode = Mode.LINEAR;
                    }
                    break;

                case LINEAR:


                    if (sharePressed()){
                        mode = Mode.BASIC;
                    }
                    break;
            }


            telemetry.addLine("<font color=#19cce8>switch modes by pressing back/share</font>");
            telemetry.addData("<font color=Magenta>mode</font>", mode);

            actuator.displayDebugInfo(telemetry);
            telemetry.addData("average loop time", "%.1f",timeUtil.getAverageLoopTime());

            telemetry.update();
        }
    }
    // So I don't have to write this every state switch
    boolean sharePressed(){return currentGamepad.share && !prevGamepad.share;}
}