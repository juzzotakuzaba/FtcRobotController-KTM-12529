package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RedCarousel extends Robot {
    OpenCvCamera webcam;
    TouchSensor touch;
    DistanceSensor distancion;

    EasyOpenCVVision pipeline;
    private ElapsedTime lifttime = new ElapsedTime(5);

    final double COUNTS_PER_INCH = 17.1;


    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);







        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distancion = hardwareMap.get(DistanceSensor.class, "Distance");
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");


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
        sleep(100);


        telemetry.addLine("Webcam ready for start");

        telemetry.addData("Number of rings ", pipeline.position);
        telemetry.addData("avg1", dataFromOpenCV.AVG1);
        telemetry.addData("avg2", dataFromOpenCV.AVG2);

        telemetry.update();

        s3Rotation.setPosition(0);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);





















        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = drive.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = drive.trajectoryBuilder(first.end(), true)
                .lineToConstantHeading(new Vector2d(-15, -40))
                .build();

        Trajectory second = drive.trajectoryBuilder(firstt.end(), true)
                .lineToConstantHeading(new Vector2d(5, 15))
                .build();

        Trajectory secondd = drive.trajectoryBuilder(firstt.end(), true)
                .splineTo(
                        new Vector2d(5, 15), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



        waitForStart();
        {
            webcam.stopStreaming();
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.addData("avg1", dataFromOpenCV.AVG1);
            telemetry.addData("avg2", dataFromOpenCV.AVG2);
            telemetry.update();
            //TODO
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
            boolean check = true;












            drive.followTrajectory(firstt);
            sleep(1000);
            drive.followTrajectory(secondd);
            carousel();
        }
    }
}
