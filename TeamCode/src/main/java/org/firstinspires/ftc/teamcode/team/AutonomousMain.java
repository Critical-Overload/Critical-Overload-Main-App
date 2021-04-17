package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "AutonomousMain")
public class AutonomousMain extends LinearOpMode {
    double hue;
    OpenCvCamera webcam;
    MainPipeline mainPipeline;
    double sensitivity;

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robotControl robot = new robotControl(hardwareMap);

        Pose2d startPose = new Pose2d(-62, 49, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory startTrajectory = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-20,49),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-1,37), Math.toRadians(0))
                .build();

        Trajectory shootingTrajectory = drive.trajectoryBuilder(startTrajectory.end())
                .addDisplacementMarker(0,() -> {
                    robot.intakePower(1);
                    robot.outtakeHigh();
                 })
                .splineToConstantHeading(new Vector2d(-20,37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-1,37), Math.toRadians(0))
                .build();

        Trajectory Atrajectory = drive.trajectoryBuilder(shootingTrajectory.end())
                .splineToConstantHeading(new Vector2d(-5,62),Math.toRadians(0))
                .addDisplacementMarker(14,() -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                    robot.retractWobbleArm();
                })
                .splineToSplineHeading(new Pose2d(-35,35, Math.toRadians(225)), Math.toRadians(0))
                .addDisplacementMarker(80, () -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.closeClaw();
                })
                .splineToSplineHeading(new Pose2d(-5,62, Math.toRadians(0)), Math.toRadians(225))
                .addDisplacementMarker(() -> {
                   robot.openClaw();
                   robot.retractWobbleArm();
                })
                .splineToConstantHeading(new Vector2d(11,62), Math.toRadians(0))
                .build();

        Trajectory Btrajectory = drive.trajectoryBuilder(shootingTrajectory.end())
                .splineToConstantHeading(new Vector2d(30,37), Math.toRadians(0))
                .addDisplacementMarker(15, () -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                    robot.retractWobbleArm();
                })
                .splineToSplineHeading(new Pose2d(-35,35,Math.toRadians(225)), Math.toRadians(0))
                .addDisplacementMarker(75, () -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.closeClaw();
                })
                .splineToSplineHeading(new Pose2d(30,37,Math.toRadians(0)), Math.toRadians(225))
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                    robot.retractWobbleArm();
                })
                .splineToConstantHeading(new Vector2d(11,37), Math.toRadians(0))
                .build();

        Trajectory Ctrajectory = drive.trajectoryBuilder(shootingTrajectory.end())
                .splineToSplineHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(0))
                .addDisplacementMarker(28, () -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                    robot.retractWobbleArm();
                })
                .splineToSplineHeading(new Pose2d(-35,35, Math.toRadians(225)), Math.toRadians(45))
                .addDisplacementMarker(125, () -> {
                    robot.extendWobbleArm();
                })
                .addDisplacementMarker(() -> {
                    robot.closeClaw();

                })
                .splineToSplineHeading(new Pose2d(50,50,Math.toRadians(45)), Math.toRadians(225))
                .addDisplacementMarker(() -> {
                    robot.openClaw();
                    robot.retractWobbleArm();
                })
                .splineToConstantHeading(new Vector2d(11,50), Math.toRadians(225))
                .build();

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        webcam.openCameraDevice();

        mainPipeline = new MainPipeline();
        webcam.setPipeline(mainPipeline);


        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("The Robot is Ready");
        telemetry.update();

        //Input Upright Mid Point: 240,320
        //Input Sideways Mid Point: 320,240

        waitForStart();

        drive.followTrajectory(startTrajectory);
        robot.shoot();

        //targetZone: 1 = A, 2 = B, 3 = C
        int targetZone = 0;
        int stackThreshold = 60;

        int stack = mainPipeline.ycontours.size();
        telemetry.addData("Stack Height before case: ", mainPipeline.stackHeight);

        if (mainPipeline.stackHeight < 70) {
            targetZone = 1;

        } else if (mainPipeline.stackHeight > 120) {
            targetZone = 3;

        } else {
            targetZone = 2;

        }

        drive.followTrajectory(shootingTrajectory);
        robot.shoot();
        robot.intakeOff();
        robot.outtakeOff();

        telemetry.addData("Stack Height: ", mainPipeline.stackHeight);
        telemetry.addData("tz: ", targetZone);
        telemetry.update();

        switch(targetZone){
            case 1:
                drive.followTrajectory(Atrajectory);
                break;
            case 2:
                drive.followTrajectory(Btrajectory);
                break;
            case 3:
                drive.followTrajectory(Ctrajectory);
                break;
            default:
                telemetry.addLine("Did Not Read Stack");
                break;
        }

    }

    class MainPipeline extends OpenCvPipeline
    {
        List<MatOfPoint> ycontours = new ArrayList<>();

        int stackHeight;

        Mat hsvImage = new Mat();
        Mat blurImg = new Mat();
        Mat output = new Mat();
        Mat yellow = new Mat();
        Mat hierachy = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            input.copyTo(output);

            ycontours.clear();

            //yellow = 60
            //Blue = 240
            //red = 0 or 360
            hue = 33;
            sensitivity = 20;


            //blur image
            Imgproc.GaussianBlur(input, blurImg, new Size(5, 5), 0);

            //converting blurred image from BGR to HSV
            Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_RGB2HSV);

            //find yellow contours
            Core.inRange(hsvImage, new Scalar((hue / 2) - sensitivity, 100, 50), new Scalar((hue / 2) + sensitivity, 255, 255), yellow);
            Imgproc.findContours(yellow, ycontours, hierachy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            if (ycontours.size() > 0){
                double ymaxVal = 0;
                int ymaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < ycontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(ycontours.get(contourIdx));
                    if (ymaxVal < contourArea) {
                        ymaxVal = contourArea;
                        ymaxValIdx = contourIdx;
                    }
                }
                //Find the bounding box of the largest yellow contour
                Rect ylargestRect = Imgproc.boundingRect(ycontours.get(ymaxValIdx));
                Imgproc.rectangle(output, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 255, 255), -1, 8, 0);

                stackHeight = ylargestRect.height;
            }


            return output;
        }

    }

}