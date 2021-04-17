package org.firstinspires.ftc.teamcode.team;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class robotControl {
    public DcMotor intake;
    public DcMotor outtakeLeft;
    public DcMotor outtakeRight;
    private DcMotor wobbleArm;

    //Declare servos
    private Servo flipper;
    private Servo wobbleClaw;

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

    }

    public void AutoStartup(){
        wobbleClaw.setPosition(0);
        flipper.setPosition(1);
    }

    public void outtakeHigh(){
        outtakeLeft.setPower(0.64);
        outtakeRight.setPower(0);
    }

    public void outtakePower(){
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
}

