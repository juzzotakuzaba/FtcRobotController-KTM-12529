package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RoadRunnerBlueUtki extends LinearOpMode {
    OpenCvCamera webcam;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        EasyOpenCVVision pipeline = new EasyOpenCVVision();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo s3Rotation = null;
        s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");

        DcMotor m6Intake = null;
        DcMotor m5Lift = null;
        DcMotor m7carousel=null;
        DcMotor m8Val=null;

        m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        m7carousel = hardwareMap.get(DcMotor.class, "m7 rul");
        m8Val = hardwareMap.get(DcMotor.class, "m8 Val");




        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = drive.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = drive.trajectoryBuilder(first.end(), true)
                .lineToConstantHeading(new Vector2d(-12, 40))
                .build();

        Trajectory second = drive.trajectoryBuilder(firstt.end(), true)
                .lineToConstantHeading(new Vector2d(7, -13))
                .build();

        Trajectory secondd = drive.trajectoryBuilder(second.end(), true)
                .strafeRight(2)
                .build();

        Trajectory seconddd = drive.trajectoryBuilder(secondd.end(), true)
                .forward(1)
                .build();

        Trajectory third = drive.trajectoryBuilder(seconddd.end(), true)
                .strafeLeft(5)
                .forward(3)
                .strafeRight(4)
                .build();

        Trajectory firsttwo = drive.trajectoryBuilder(third.end(), true)
                .lineToConstantHeading(new Vector2d(-12, 40))
                .build();

        Trajectory third1 = drive.trajectoryBuilder(second.end(), true)
                .lineToConstantHeading(new Vector2d(-16, -22))
                .build();


        waitForStart();
        {
            int ShElementPosition = 10;
            if ((pipeline.position == EasyOpenCVVision.ShipPosition.LEFT)) {
                ShElementPosition = 3;
            }
            if ((pipeline.position == EasyOpenCVVision.ShipPosition.CENTER)) {
                ShElementPosition = 2;
            }
            if ((pipeline.position == EasyOpenCVVision.ShipPosition.NONE)) {
                ShElementPosition = 1;
            }
            //Voltage regulation depending on the battery charge level
            telemetry.addData("ShElementPosition", ShElementPosition);

            telemetry.update();



            sleep(4000);

            drive.followTrajectory(firstt);


            //3 этаж, право
            if (ShElementPosition==1) {
            m5Lift.setPower(-1);
            sleep(1250);
            m5Lift.setPower(0);
            s3Rotation.setPosition(0.73);
            sleep(1350);
            s3Rotation.setPosition(0);
            m5Lift.setPower(1);
            sleep(1200);
            m5Lift.setPower(0);}

            //2 этаж, центр
            if (ShElementPosition==2) {
                m5Lift.setPower(-1);
                sleep(680);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1350);
                s3Rotation.setPosition(0);
                m5Lift.setPower(1);
                sleep(644);
                m5Lift.setPower(0);}

            //1 этаж
            if (ShElementPosition==3) {
                m5Lift.setPower(-1);
                sleep(350);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1444);
                s3Rotation.setPosition(0);
                m5Lift.setPower(1);
                sleep(300);
                m5Lift.setPower(0);}



            drive.followTrajectory(second);
            drive.followTrajectory(secondd);
            drive.followTrajectory(seconddd);
            m7carousel.setPower(0.33);
            sleep(2000);
            m7carousel.setPower(0);
            m6Intake.setPower(-0.7);
            m8Val.setPower(-1);
            drive.followTrajectory(third);
            drive.followTrajectory(firsttwo);
             /* {
                m5Lift.setPower(-1);
                sleep(1250);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1350);
                s3Rotation.setPosition(0);
                m5Lift.setPower(1);
                sleep(1200);
                m5Lift.setPower(0);}*/
            drive.followTrajectory(third1);
        }
    }
}
