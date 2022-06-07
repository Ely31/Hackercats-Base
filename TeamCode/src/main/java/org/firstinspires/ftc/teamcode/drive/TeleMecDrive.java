package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AutoToTele;

public class TeleMecDrive {
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    private BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    public double heading;

    private double rotX;
    private double rotY;

    private double slowFactor;

    TeleMecDrive(HardwareMap hardwareMap, double slowFactor) {
        lf = hardwareMap.get(DcMotor.class,"lf");
        lb = hardwareMap.get(DcMotor.class,"lb");
        rf = hardwareMap.get(DcMotor.class,"rf");
        rb = hardwareMap.get(DcMotor.class,"rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        //brake when you stop gamepad input
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        this.slowFactor = slowFactor;
    }

    public void driveFieldCentric(double x, double y, double turn, double slowInput){

        slowInput = ((-1 + slowFactor) * slowInput)+1;

        heading = -(imu.getAngularOrientation().firstAngle + (AutoToTele.endOfAutoHeading-Math.toRadians(90 * AutoToTele.allianceSide)));

        rotX = x * Math.cos(heading) - -y * Math.sin(heading);
        rotY = x * Math.sin(heading) + -y * Math.cos(heading);

        double lfPower = rotY + rotX + turn;
        double lbPower = rotY - rotX + turn;
        double rfPower = rotY - rotX - turn;
        double rbPower = rotY + rotX - turn;

        lf.setPower(lfPower*slowInput);
        lb.setPower(lbPower*slowInput);
        rf.setPower(rfPower*slowInput);
        rb.setPower(rbPower*slowInput);
    }

    public void driveRobotCentric(double x, double y, double turn, double slowInput){
        slowInput = ((-1 + slowFactor) * slowInput)+1;

        double lfPower = y + x + turn;
        double lbPower = y - x + turn;
        double rfPower = y - x - turn;
        double rbPower = y + x - turn;

        lf.setPower(lfPower*slowInput);
        lb.setPower(lbPower*slowInput);
        rf.setPower(rfPower*slowInput);
        rb.setPower(rbPower*slowInput);
    }

    public void resetHeading(){
        AutoToTele.endOfAutoHeading = (Math.PI/2)*AutoToTele.allianceSide; // Unit circle coming in handy
        imu.initialize(imuParameters);
    }

    public void setWheelPowers(double LF, double LB, double RF, double RB){
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setPower(LF);
        lb.setPower(LB);
        rf.setPower(RF);
        rb.setPower(RB);
    }
}
