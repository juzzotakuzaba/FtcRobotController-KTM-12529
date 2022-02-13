/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "RedRight", group = "AutoOP")
public class RedRight extends Robot {
    OpenCvCamera webcam;

    TouchSensor touch;
    DistanceSensor distancion;

    EasyOpenCVVision pipeline;
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    private ElapsedTime lifttime = new ElapsedTime(5);

    final double COUNTS_PER_INCH = 17.1;

    String rfName = "m4 drive", rbName = "m1 drive", lfName = "m2 drive", lbName = "m3 drive";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;


    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distancion = hardwareMap.get(DistanceSensor.class, "Distance");
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

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
        telemetry.addData("avg3", dataFromOpenCV.AVG3);
        telemetry.update();

        s3Rotation.setPosition(0);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);
        waitForStart();
        {
            m5Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m5Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ElapsedTime timer = new ElapsedTime();
            ElapsedTime timing = new ElapsedTime();
            webcam.stopStreaming();
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.addData("avg1", dataFromOpenCV.AVG1);
            telemetry.addData("avg2", dataFromOpenCV.AVG2);
            telemetry.update();
            //TODO
            //sleep(10000);
            double dalnost = distancion.getDistance(DistanceUnit.CM);
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

            //lifttime.reset();


            if (ShElementPosition == 1 && check) {
                m5Lift.setPower(-1);
                s3Rotation.setPosition(0.3);

                setMotorsPower(0 * koeff, 0.4 * koeff, 0.4 * koeff, 0 * koeff);
                sleep(500);
                setMotorsPower(-0.42 * koeff, 0.42 * koeff, 0.42 * koeff, -0.42 * koeff);
                sleep(800);

                setMotorsPower(0 * koeff, 0 * koeff, 0 * koeff, 0 * koeff);
                //m5Lift.setPower(0.8);



                for (int i = 0; i < 2; i = i + 1) {

                    m5Lift.setPower(0);
                    sleep(200);
                    s3Rotation.setPosition(0.73);
                    sleep(1100);
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0);
                    sleep(700);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(550);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(150);

                    setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                    sleep(700);

                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);

                    setMotorsPower(0, 0, 0, 0);

                    m6Intake.setPower(-0.9);
                    m8Val.setPower(-1);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.5 * koeff, 0.5 * koeff);
                    sleep(570);
                    setMotorsPower(0, 0, 0, 0);
                    setMotorsPower(0.45 * koeff, -0.45 * koeff, -0.2 * koeff, 0.2 * koeff);
                    sleep(1100);
                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);
                    m6Intake.setPower(0.3);
                    m8Val.setPower(0.8);

                   // boolean flag = false;
                    //if (distancion.getDistance(DistanceUnit.CM) > 200) {
                        //flag = true;
                    //}
                    //timing.reset();
                    //while (!flag && distancion.getDistance(DistanceUnit.CM) < 50) {
                        //telemetry.addData("millis", timing.milliseconds());
                        //telemetry.update();
                    setMotorsPower(-0.1 * koeff, 0.1 * koeff, 0.5 * koeff, -0.5 * koeff);
                    //}
                    sleep(600);

                    //if(flag){
                    //}


                    setMotorsPower(-0.2 * koeff, 0.2 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(1200);
                    setMotorsPower(0, 0, 0, 0);
                    m6Intake.setPower(0);
                    m8Val.setPower(0);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(1300);
                    m5Lift.setPower(-1);
                    s3Rotation.setPosition(0.3);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(400);
                    setMotorsPower(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff);
                    sleep(780);
                    setMotorsPower(0, 0, 0, 0);
                    m5Lift.setPower(0);
                }
            }


            if (ShElementPosition == 2 && check) {


                setMotorsPower(0 * koeff, 0.4 * koeff, 0.4 * koeff, 0 * koeff);
                sleep(500);

                setMotorsPower(-0.42 * koeff, 0.42 * koeff, 0.42 * koeff, -0.42 * koeff);
                sleep(150);

                m5Lift.setPower(-1);
                s3Rotation.setPosition(0.3);

                setMotorsPower(-0.42 * koeff, 0.42 * koeff, 0.42 * koeff, -0.42 * koeff);
                sleep(650);

                setMotorsPower(0 * koeff, 0 * koeff, 0 * koeff, 0 * koeff);
                //m5Lift.setPower(0.8);




                {m5Lift.setPower(0);
                    sleep(100);
                    s3Rotation.setPosition(0.73);
                    sleep(1100);
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0);
                    sleep(650);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(550);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(150);

                    setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                    sleep(700);

                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);

                    setMotorsPower(0, 0, 0, 0);

                    m6Intake.setPower(-0.9);
                    m8Val.setPower(-1);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.5 * koeff, 0.5 * koeff);
                    sleep(470);
                    setMotorsPower(0, 0, 0, 0);
                    setMotorsPower(0.45 * koeff, -0.45 * koeff, -0.2 * koeff, 0.2 * koeff);
                    sleep(1100);
                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);
                    m6Intake.setPower(0.3);
                    m8Val.setPower(0.8);


                    setMotorsPower(-0.1 * koeff, 0.1 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(600);


                    setMotorsPower(-0.2 * koeff, 0.2 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(800);
                    setMotorsPower(0, 0, 0, 0);
                    m6Intake.setPower(0);
                    m8Val.setPower(0);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(1000);
                    m5Lift.setPower(-1);
                    s3Rotation.setPosition(0.33);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(400);
                    setMotorsPower(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff);
                    sleep(830);
                    setMotorsPower(0, 0, 0, 0);
                    m5Lift.setPower(0);}
                {m5Lift.setPower(0);
                    sleep(100);
                    s3Rotation.setPosition(0.73);
                    sleep(1100);
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0);
                    sleep(650);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(550);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(150);

                    setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                    sleep(700);

                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);

                    setMotorsPower(0, 0, 0, 0);

                    m6Intake.setPower(-0.9);
                    m8Val.setPower(-1);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.5 * koeff, 0.5 * koeff);
                    sleep(470);
                    setMotorsPower(0, 0, 0, 0);
                    setMotorsPower(0.45 * koeff, -0.45 * koeff, -0.2 * koeff, 0.2 * koeff);
                    sleep(1100);
                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);
                    m6Intake.setPower(0.3);
                    m8Val.setPower(0.8);

                    setMotorsPower(-0.1 * koeff, 0.1 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(600);


                    setMotorsPower(-0.2 * koeff, 0.2 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(800);
                    setMotorsPower(0, 0, 0, 0);
                    m6Intake.setPower(0);
                    m8Val.setPower(0);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(1000);
                    m5Lift.setPower(-1);
                    s3Rotation.setPosition(0.33);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(400);
                    setMotorsPower(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff);
                    sleep(830);
                    setMotorsPower(0, 0, 0, 0);
                    m5Lift.setPower(0);}
            }


            if (ShElementPosition == 3 && check) {


                setMotorsPower(0 * koeff, 0.4 * koeff, 0.4 * koeff, 0 * koeff);
                sleep(500);

                setMotorsPower(-0.42 * koeff, 0.42 * koeff, 0.42 * koeff, -0.42 * koeff);
                sleep(450);

                m5Lift.setPower(-1);
                s3Rotation.setPosition(0.3);

                setMotorsPower(-0.42 * koeff, 0.42 * koeff, 0.42 * koeff, -0.42 * koeff);
                sleep(350);

                setMotorsPower(0 * koeff, 0 * koeff, 0 * koeff, 0 * koeff);
                //m5Lift.setPower(0.8);




                {

                    m5Lift.setPower(0);
                    sleep(100);
                    s3Rotation.setPosition(0.73);
                    sleep(1100);
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0);
                    sleep(300);
                        m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(550);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(150);

                    setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                    sleep(700);

                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);

                    setMotorsPower(0, 0, 0, 0);

                    m6Intake.setPower(-0.9);
                    m8Val.setPower(-1);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.5 * koeff, 0.5 * koeff);
                    sleep(470);
                    setMotorsPower(0, 0, 0, 0);
                    setMotorsPower(0.45 * koeff, -0.45 * koeff, -0.2 * koeff, 0.2 * koeff);
                    sleep(1100);
                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);
                    m6Intake.setPower(0.3);
                    m8Val.setPower(0.8);

                    setMotorsPower(-0.1 * koeff, 0.1 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(600);



                    setMotorsPower(-0.2 * koeff, 0.2 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(800);
                    setMotorsPower(0, 0, 0, 0);
                    m6Intake.setPower(0);
                    m8Val.setPower(0);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(1000);
                    m5Lift.setPower(-1);
                    s3Rotation.setPosition(0.33);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(400);
                    setMotorsPower(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff);
                    sleep(830);
                    setMotorsPower(0, 0, 0, 0);
                    m5Lift.setPower(0);
                }

                {

                    m5Lift.setPower(0);
                    sleep(100);
                    s3Rotation.setPosition(0.73);
                    sleep(1100);
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0);
                    sleep(620);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(550);
                    m5Lift.setPower(0);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                    sleep(150);

                    setMotorsPower(0.3 * koeff, 0.3 * koeff, 0.3 * koeff, 0.3 * koeff);
                    sleep(700);

                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);

                    setMotorsPower(0, 0, 0, 0);

                    m6Intake.setPower(-0.9);
                    m8Val.setPower(-1);
                    setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.5 * koeff, 0.5 * koeff);
                    sleep(470);
                    setMotorsPower(0, 0, 0, 0);
                    setMotorsPower(0.45 * koeff, -0.45 * koeff, -0.2 * koeff, 0.2 * koeff);
                    sleep(1100);
                    setMotorsPower(0.3 * koeff, -0.3 * koeff, 0.3 * koeff, -0.3 * koeff);
                    sleep(1000);
                    m6Intake.setPower(0.3);
                    m8Val.setPower(0.8);

                    setMotorsPower(-0.1 * koeff, 0.1 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(600);



                    setMotorsPower(-0.2 * koeff, 0.2 * koeff, 0.5 * koeff, -0.5 * koeff);
                    sleep(800);
                    setMotorsPower(0, 0, 0, 0);
                    m6Intake.setPower(0);
                    m8Val.setPower(0);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(1000);
                    m5Lift.setPower(-1);
                    s3Rotation.setPosition(0.33);
                    setMotorsPower(-0.33 * koeff, 0 * koeff, 0 * koeff, -0.33 * koeff);
                    sleep(400);
                    setMotorsPower(-0.4 * koeff, 0.4 * koeff, 0.4 * koeff, -0.4 * koeff);
                    sleep(830);
                    setMotorsPower(0, 0, 0, 0);
                    m5Lift.setPower(0);
                }

            }




                s3Rotation.setPosition(0.73);
                sleep(800);

                setMotorsPower(0.7 * koeff, -0.7 * koeff, -0.1 * koeff, 0.1 * koeff);
                sleep(100);

                m5Lift.setPower(1);
                s3Rotation.setPosition(0);
                if (ShElementPosition == 1) {
                setMotorsPower(0.2 * koeff, 0.2 * koeff, 0.2 * koeff, 0.2 * koeff);
                sleep(1050);}
                if (ShElementPosition == 2) {
                setMotorsPower(0.2 * koeff, 0.2 * koeff, 0.2 * koeff, 0.2 * koeff);
                sleep(800);}
                if (ShElementPosition == 3) {
                setMotorsPower(0.2 * koeff, 0.2 * koeff, 0.2 * koeff, 0.2 * koeff);
                sleep(800);}
                setMotorsPower(0, 0, 0, 0);
                setMotorsPower(1 * koeff, -1 * koeff, -1 * koeff, 1 * koeff);
                sleep(300);
                m5Lift.setPower(0);
                sleep(600);

                setMotorsPower(0 * koeff, 0 * koeff, 0 * koeff, 0 * koeff);


            //добавить координату



        }

    }



    protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        right_back.setPower(D1_power);
        left_front.setPower(D2_power);
        left_back.setPower(D3_power);
        right_front.setPower(D4_power);
    }

    protected void stopMovement() {
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }

    }
}