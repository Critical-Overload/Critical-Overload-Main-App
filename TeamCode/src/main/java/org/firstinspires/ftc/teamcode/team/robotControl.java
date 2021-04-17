package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class robotControl {
    public DcMotor intake;
    public DcMotor outtakeLeft;
    public DcMotor outtakeRight;
    private DcMotor wobbleArm;

    //Declare servos
    private Servo flipper;
    private Servo wobbleClaw;

    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    public robotControl(HardwareMap hardwareMap){
        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");

        flipper = hardwareMap.servo.get("flipper");
        intake = hardwareMap.dcMotor.get("intake");

        outtakeLeft=hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight=hardwareMap.get(DcMotor.class, "outtakeRight");

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void AutoStartup(){
        wobbleClaw.setPosition(0);
        flipper.setPosition(1);
    }

    public void outtakeHigh(){
        outtakeLeft.setPower(0.64);
        outtakeRight.setPower(0);
    }

    public void outtakePowershot(){
        outtakeLeft.setPower(0.57);
        outtakeRight.setPower(0);
    }

    public void outtakeOff(){
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    public void shoot(){
        for (int i = 0; i < 3; i++){
            flipper.setPosition(0);
            flipper.setPosition(1);
        }
    }

    public void intakePower (double power) {
        intake.setPower(power);
    }

    public void outtakePower (double power){
        outtakeLeft.setPower(power);
    }

    public void intakeOff(){
        intakePower(0);
    }

    public void extendWobbleArm(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < 1500){
            wobbleArm.setPower(-.5);
        }
        wobbleArm.setPower(0);
    }

    public void retractWobbleArm(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 1500) {
            wobbleArm.setPower(.5);
        }
        wobbleArm.setPower(0);
    }

    public void openClaw(){
        wobbleClaw.setPosition(1);
    }

    public void closeClaw(){
        wobbleClaw.setPosition(0);
    }

    public void shootOnce(){
        flipper.setPosition(0);
        flipper.setPosition(1);
    }

    public void teleopDrive(double powerOne, double powerTwo, double rotation, double powerMod){
        motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
        motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
        motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
        motorBackRight.setPower((powerOne + (rotation))*powerMod);
    }

    public void wobblePower(double power){
        wobbleArm.setPower(power);
    }
}
