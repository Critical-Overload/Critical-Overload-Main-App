package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
//import org.openftc.revextensions2.ExpansionHubEx;

/**
 * Main Teleop
 *
 * 3 October 2020
 */

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        robotControl robot = new robotControl(hardwareMap);

        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = .64;
        double wobbleMod = .3;

        waitForStart();


        while(opModeIsActive()){
            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            //everything intake
            /*
            Change direction of intake
            */
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }
            robot.intakePower(gamepad1.left_trigger*0.85*intakeMod);



            //Ring flipper
            //Run by a servo, 1 is fully "flipped" position, 0 is fully "retracted" position
            //Hold down b button to flip ring out
            if(gamepad2.a){
                robot.shootOnce();
            }

            //everything driving
            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            robot.teleopDrive(powerOne,powerTwo,rotation,powerMod);

            //everything outtake/launch

            if(gamepad2.right_bumper){
                outtakeMod= .57;
                //outtakeMod = 0.32; //power shots
            }else{
                outtakeMod=.64;
                // outtakeMod = 0.3225;
            }

            if(gamepad2.right_trigger > .3) {
                robot.outtakePower(outtakeMod);
                robot.shootOnce();
            }
            else{
                robot.outtakePower(0);
            }

            //everything wobble

            if(gamepad2.left_bumper){
                wobbleMod = 1.0;
            }else{
                wobbleMod = .6;
            }
            robot.wobblePower(gamepad2.left_stick_y * wobbleMod );

            if(gamepad2.x){
                robot.closeClaw();
            }

            if(gamepad2.y){
                robot.openClaw();
            }

            telemetry.update();
            idle();
        }

    }

}