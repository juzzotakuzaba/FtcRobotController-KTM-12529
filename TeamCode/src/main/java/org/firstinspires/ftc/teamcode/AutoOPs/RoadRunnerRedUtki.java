package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RoadRunnerRedUtki extends LinearOpMode {
    OpenCvCamera webcam;
    TouchSensor touch;
    private ElapsedTime lifttime = new ElapsedTime(5);
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

        touch = hardwareMap.get(TouchSensor.class, "Touch");

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
                .lineToConstantHeading(new Vector2d(-16, -37))
                .build();

        Trajectory firstt2 = drive.trajectoryBuilder(firstt.end(), true)
                .back(2)
                .build();

        Trajectory second = drive.trajectoryBuilder(firstt.end(), true)
                .lineToConstantHeading(new Vector2d(6, 18))
                .build();

        Trajectory secondd = drive.trajectoryBuilder(firstt.end(), true)
                .strafeTo(new Vector2d(3,18))
                .build();

        Trajectory seconddd = drive.trajectoryBuilder(secondd.end(), true)
                .strafeLeft(5.5)
                .build();

        Trajectory seconddd1 = drive.trajectoryBuilder(seconddd.end(), true)
                .forward(3)
                .build();

        Trajectory seconddd2 = drive.trajectoryBuilder(seconddd1.end(), true)
                .strafeLeft(1)
                .build();

        Trajectory third = drive.trajectoryBuilder(seconddd2.end(), true)
                .strafeTo(new Vector2d(0,12))
                .build();

        Trajectory forr1 = drive.trajectoryBuilder(third.end(), true)
                .forward(6)
                .build();

        Trajectory forr = drive.trajectoryBuilder(forr1.end(), true)
                .strafeLeft(5)
                .build();

        Trajectory firsttwo = drive.trajectoryBuilder(forr.end(), true)
                .strafeTo(new Vector2d(-16,-37))
                .build();

        Trajectory firsttw0 = drive.trajectoryBuilder(firsttwo.end(), true)
                .back(1)
                .build();

        Trajectory third1 = drive.trajectoryBuilder(firsttw0.end(), true)
                .lineToConstantHeading(new Vector2d(-15, 16))
                .build();


        double BatteryVoltage;

        double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }


        double voltage = result;
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);

        waitForStart();
        {
            int downpos = 1;
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




            sleep(50);

            drive.followTrajectory(firstt);


            //3 этаж, право
            if (ShElementPosition==1) {
            m5Lift.setPower(-1);
            sleep(1250);
            m5Lift.setPower(0);
            s3Rotation.setPosition(0.73);
            sleep(1350);
            s3Rotation.setPosition(0);
            lifttime.reset();
            while (!touch.isPressed()&&lifttime.milliseconds() < 1180)  {
                if (touch.isPressed()) downpos = downpos + 1;
                m5Lift.setPower(1);
            }
            m5Lift.setPower(0);}

            //2 этаж, центр
            if (ShElementPosition==2) {
                m5Lift.setPower(-1);
                sleep(680);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1350);
                s3Rotation.setPosition(0);
                lifttime.reset();
                while (!touch.isPressed()&&lifttime.milliseconds() < 650)  {
                    if (touch.isPressed()) downpos = downpos + 1;
                    m5Lift.setPower(1);
                }
                m5Lift.setPower(0);}

            //1 этаж
            if (ShElementPosition==3) {
                m5Lift.setPower(-1);
                sleep(350);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1444);
                s3Rotation.setPosition(0);
                lifttime.reset();
                while (!touch.isPressed()&&lifttime.milliseconds() < 350)  {
                    if (touch.isPressed()) downpos = downpos + 1;
                    m5Lift.setPower(1);
                }
                m5Lift.setPower(0);}



            //drive.followTrajectory(second);
            drive.followTrajectory(secondd);
            m7carousel.setPower(-0.3);
            drive.followTrajectory(seconddd);
            drive.followTrajectory(seconddd1);
            drive.followTrajectory(seconddd2);
            sleep(2100);
            m7carousel.setPower(0);
            m6Intake.setPower(-0.7);
            m8Val.setPower(-1);
            drive.followTrajectory(third);
            drive.followTrajectory(forr1);
            drive.followTrajectory(forr);
            drive.followTrajectory(firsttwo);
            drive.followTrajectory(firsttw0);

            m6Intake.setPower(0);
            m8Val.setPower(0);
             {
                m5Lift.setPower(-1);
                sleep(1250);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1350);
                s3Rotation.setPosition(0);
                lifttime.reset();
                while (!touch.isPressed()&&lifttime.milliseconds() < 1130)  {
                     m5Lift.setPower(1);
                    if (touch.isPressed()) m5Lift.setPower(0);}
                m5Lift.setPower(0);}
            //drive.followTrajectory(third1);

            drive.setMotorPowers(0.2*koeff,0.2*koeff,0.2*koeff,0.2*koeff);
            sleep(222);

            if (ShElementPosition==1){

            drive.setMotorPowers(-0.3*koeff,-0.3*koeff,0.3*koeff,0.3*koeff);
            sleep(940);

            m6Intake.setPower(-0.6);
            m8Val.setPower(-1);

            drive.setMotorPowers(0.4*koeff,0.4*koeff,0.4*koeff,0.4*koeff);
            sleep(888);

            drive.setMotorPowers(0.3*koeff,0.3*koeff,-0.3*koeff,-0.3*koeff);
            sleep(100);



            drive.setMotorPowers(0.3*koeff,0.3*koeff,-0.3*koeff,-0.3*koeff);
            sleep(440);

            drive.setMotorPowers(-0.4*koeff,-0.4*koeff,-0.4*koeff,-0.4*koeff);
            sleep(380);

            drive.setMotorPowers(0,0,0,0);
            sleep(400);

            m6Intake.setPower(0.3);
            m8Val.setPower(1);

            drive.setMotorPowers(0,0,0,0);
            {
                m5Lift.setPower(-1*koeff);
                sleep(1250);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0.73);
                sleep(1350);
                s3Rotation.setPosition(0);
                m5Lift.setPower(1);
            }
            m6Intake.setPower(0);
            m8Val.setPower(0);

            drive.setMotorPowers(0.4*koeff,0.4*koeff,0.4*koeff,0.4*koeff);
            sleep(400);

            drive.setMotorPowers(-0.4*koeff,-0.4*koeff,0.4*koeff,0.4*koeff);
            sleep(280);

            drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
            sleep(100);

            m5Lift.setPower(0);

            drive.setMotorPowers(1,1,1,1);
            sleep(1000);
            }

            if (ShElementPosition==2){

                drive.setMotorPowers(-0.3*koeff,-0.3*koeff,0.3*koeff,0.3*koeff);
                sleep(900);

                m6Intake.setPower(-0.6);
                m8Val.setPower(-1);

                drive.setMotorPowers(0.4*koeff,0.4*koeff,0.4*koeff,0.4*koeff);
                sleep(700);

                drive.setMotorPowers(0,0,0,0);
                sleep(100);

                drive.setMotorPowers(-0.3*koeff,-0.3*koeff,0.3*koeff,0.3*koeff);
                sleep(100);

                drive.setMotorPowers(0.3*koeff,0.3*koeff,-0.3*koeff,-0.3*koeff);
                sleep(610);

                drive.setMotorPowers(-0.4*koeff,-0.4*koeff,-0.4*koeff,-0.4*koeff);
                sleep(210);

                drive.setMotorPowers(0,0,0,0);
                sleep(100);
                {
                    m5Lift.setPower(-1*koeff);
                    sleep(1250);
                    m5Lift.setPower(0);
                    s3Rotation.setPosition(0.73);
                    sleep(1350);
                    s3Rotation.setPosition(0);
                    m5Lift.setPower(1);
                }
                m6Intake.setPower(0);
                m8Val.setPower(0);

                drive.setMotorPowers(0.4*koeff,0.4*koeff,0.4*koeff,0.4*koeff);
                sleep(200);

                drive.setMotorPowers(-0.4*koeff,-0.4*koeff,0.4*koeff,0.4*koeff);
                sleep(400);

                drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
                sleep(500);


                drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
                sleep(100);
                m5Lift.setPower(0);
                drive.setMotorPowers(0.5*koeff,-0.5*koeff,0.5*koeff,-0.5*koeff);
                sleep(900);

                drive.setMotorPowers(0.7,0.7,0.5,0.5);
                sleep(580);}

            if (ShElementPosition==3){


                drive.setMotorPowers(-0.3*koeff,-0.3*koeff,0.3*koeff,0.3*koeff);
                sleep(1000);

                m6Intake.setPower(-0.6);
                m8Val.setPower(-1);

                drive.setMotorPowers(0.4*koeff,0.4*koeff,0.4*koeff,0.4*koeff);
                sleep(400);

                drive.setMotorPowers(0,0,0,0);
                sleep(300);

                drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
                sleep(80);

                drive.setMotorPowers(0.3*koeff,0.3*koeff,-0.3*koeff,-0.3*koeff);
                sleep(800);

                drive.setMotorPowers(-0.4*koeff,-0.4*koeff,-0.4*koeff,-0.4*koeff);
                sleep(100);

                drive.setMotorPowers(0,0,0,0);
                {
                    m5Lift.setPower(-1*koeff);
                    sleep(1250);
                    m5Lift.setPower(0);
                    s3Rotation.setPosition(0.73);
                    sleep(1350);
                    s3Rotation.setPosition(0);
                    m5Lift.setPower(1);
                }


                m6Intake.setPower(0);
                m8Val.setPower(0);



                drive.setMotorPowers(0.3*koeff,0.3*koeff,0.3*koeff,0.3*koeff);
                sleep(500);
                drive.setMotorPowers(-0.4*koeff,-0.4*koeff,0.4*koeff,0.4*koeff);
                sleep(400);
                drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
                sleep(300);
                m5Lift.setPower(0);
                drive.setMotorPowers(0.4*koeff,-0.4*koeff,0.4*koeff,-0.4*koeff);
                sleep(1111);
                drive.setMotorPowers(0.7*koeff,0.7*koeff,0.5*koeff,0.5*koeff);
                sleep(800);}



            telemetry.addData("DownPos", downpos);
            telemetry.update();
        }


    }
}
