


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "KTM TeleOp 2022", group = "Linear Opmode")

//@Disabled
public class TeleOP2022 extends LinearOpMode {
    private static final int LED_CHANNEL = 5;
    //    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lifttime = new ElapsedTime(5);



    //private DcMotor m6Intake = null;
    private DcMotor m7carousel = null;
    private DcMotor m5Lift = null;

    //-------
    double magic(double input) {
        if(Math.abs(input)<0.02){

            double nol=0;
            return nol;
        } else{
            return Math.signum(input) * (0.9* Math.pow(Math.abs(input), 2)+0.1);
        }
    }
    protected double BatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }








     protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power, DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
         // Send power to wheels
         right_back.setPower(D1_power);
         left_front.setPower(D2_power);
         left_back.setPower(D3_power);
         right_front.setPower(D4_power);
     }
    protected void stopMovement(DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front){
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }

 //   TouchSensor touchsensor;
      DistanceSensor distance;
      TouchSensor touch;

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * End of functions declaration
     */
    final double COUNTS_PER_INCH = 307.699557;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Chassis
        DcMotor m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        DcMotor m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        DcMotor m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        DcMotor m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        DcMotorEx m5Lift = hardwareMap.get(DcMotorEx.class, "m5 lift");
        DcMotor m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        DcMotorEx m7carousel = hardwareMap.get(DcMotorEx.class, "m7 rul");
        DcMotor m8Val = hardwareMap.get(DcMotor.class, "m8 Val");
        //  Servo s1RelicExtRet = hardwareMap.get(Servo.class, "s1 top claw");
        //s2_bottom_Claw = hardwareMap.get(CRServo.class, "s2 bottom claw");
        Servo s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");
        // Servo s4Kicker = hardwareMap.get(Servo.class, "s4 kick");
        Servo s5Shovel = hardwareMap.get(Servo.class, "s5 shovel");
        //  Servo s6RelicClaw = hardwareMap.get(Servo.class, "s6 relic claw");
        //  Servo s7RelicArm = hardwareMap.get(Servo.class, "s7 relic arm");
       // touchsensor = hardwareMap.touchSensor.get("touch_sensor");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //m1Drive.setDirection(DcMotor.Direction.FORWARD);
        //m2Drive.setDirection(DcMotor.Direction.FORWARD);
        //m3Drive.setDirection(DcMotor.Direction.FORWARD);
        //m4Drive.setDirection(DcMotor.Direction.FORWARD);
        m1Drive.setDirection(DcMotor.Direction.REVERSE);
        m2Drive.setDirection(DcMotor.Direction.REVERSE);
        m3Drive.setDirection(DcMotor.Direction.REVERSE);
        m4Drive.setDirection(DcMotor.Direction.REVERSE);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m8Val.setDirection(DcMotor.Direction.FORWARD);
        m5Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m6Intake.setDirection(DcMotor.Direction.FORWARD);
        m7carousel.setDirection(DcMotor.Direction.FORWARD);
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m8Val.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();
        runtime.reset();


        int ANDYMARK_TICKS_PER_REV = 1120;
        double m1DrivePower;
        double m2DrivePower;
        double m3DrivePower;
        double m4DrivePower;
        double m5LiftPower=0;
        double m6IntakePower=0;
        double m1m1m1=0;
        double m2m2m2=0;
        double m3m3m3=0;
        double m4m4m4=0;
        double m1m1=0;
        double m2m2=0;
        double m3m3=0;
        double m4m4=0;
        double m1left=0;
        double m1right=0;
        double m2left=0;
        double m2right=0;
        double m3left=0;
        double m3right=0;
        double m4left=0;
        double m4right=0;
        double m1DrivePowerforrotation=0;
        double m2DrivePowerforrotation=0;
        double m3DrivePowerforrotation=0;
        double m4DrivePowerforrotation=0;
        double m1DrivePowerfordrivetofoundation=0;
        double m2DrivePowerfordrivetofoundation=0;
        double m3DrivePowerfordrivetofoundation=0;
        double m4DrivePowerfordrivetofoundation=0;
        double m1DrivePowerfordrivetofoundation2=0;
        double m2DrivePowerfordrivetofoundation2=0;
        double m3DrivePowerfordrivetofoundation2=0;
        double m4DrivePowerfordrivetofoundation2=0;
        double m1DrivePowerfordrivetofoundation1=0;
        double m2DrivePowerfordrivetofoundation1=0;
        double m3DrivePowerfordrivetofoundation1=0;
        double m4DrivePowerfordrivetofoundation1=0;
        double m1DrivePowerfordrivetofoundation11=0;
        double m2DrivePowerfordrivetofoundation11=0;
        double m3DrivePowerfordrivetofoundation11=0;
        double m4DrivePowerfordrivetofoundation11=0;
        double a=0;
        double b=0;

        double seconds;
        // double prevangel=0;
        //  boolean shoot=false;
        String positionServo="not ready";

        s3Rotation.setPosition(0);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);










      /*  double handcarousel = m7carousel.getCurrentPosition();{
            {
                if (handcarousel > 0) {
                    m7carousel.setPower(handcarousel);
                }
                if (collector < 0) {
                    m7carousel.setPower(handcarousel);
                }
                else {
                    m7carousel.setPower(0);
                }
            }

        }*/








        //ТЕЛЕОП НАЧИНАЕТСЯ




        //ДВИЖЕНИЕ




        while (opModeIsActive()) {
            // POV Mode uses right stick to go forward and right to slide.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double driveL = -gamepad1.left_stick_y;
            // double driveR = -gamepad1.right_stick_y;
            // float relic = gamepad2.left_stick_x;
            //boolean ruletka_tuda = gamepad2.dpad_up;
            //boolean ruletka_suda = gamepad2.dpad_down;
            double zagrebalo = gamepad2.left_stick_y;
            //double caruselka_right = gamepad2.right_trigger;
            //double caruselka_left = gamepad2.left_trigger;
            //double podiem = 1*gamepad2.right_stick_y;
            double lift = gamepad2.right_stick_y;
            double slideR = -0.7*gamepad1.left_trigger;
            double slideL = 0.7*gamepad1.right_trigger;
            double vpernazad=gamepad1.left_stick_y;
            double vleovpravo= -gamepad1.left_stick_x;
            double povorot= gamepad1.right_stick_x;

            boolean knopka;

            ////////////////////////////////////////////////////
            //Slide Related
            slideL=magic(slideL);
            slideR=magic(slideR);
            povorot=magic(povorot);
            vpernazad=magic(vpernazad);
            vleovpravo=magic(vleovpravo);
                /*m2DrivePower = povorot-vpernazad-vleovpravo;
                m4DrivePower = povorot+vpernazad-vleovpravo;
                m1DrivePower = povorot+vpernazad+vleovpravo;
                m3DrivePower = povorot-vpernazad+vleovpravo;*/
            m2DrivePower = (m2left+m2right+m1m1+m1m1m1+m1DrivePowerfordrivetofoundation+m1DrivePowerfordrivetofoundation2+m1DrivePowerfordrivetofoundation1+m1DrivePowerfordrivetofoundation11) + povorot-vpernazad+(slideL+slideR)-(vleovpravo);
            m4DrivePower = (m4left+m4right+m2m2+m2m2m2+m2DrivePowerfordrivetofoundation+m2DrivePowerfordrivetofoundation2+m2DrivePowerfordrivetofoundation1+m2DrivePowerfordrivetofoundation11)+ povorot+vpernazad+(slideL+slideR)-(vleovpravo);
            m1DrivePower = (m1left+m1right+m3m3+m3m3m3+m3DrivePowerfordrivetofoundation+m3DrivePowerfordrivetofoundation2+m3DrivePowerfordrivetofoundation1+m3DrivePowerfordrivetofoundation11)+povorot+vpernazad+(slideL+slideR)+(vleovpravo);
            m3DrivePower = (m3left+m3right+m4m4+m4m4m4+m4DrivePowerfordrivetofoundation+m4DrivePowerfordrivetofoundation2+m4DrivePowerfordrivetofoundation1+m4DrivePowerfordrivetofoundation11)+ povorot-vpernazad+(slideL+slideR)+(vleovpravo);
            double mochs=1;
            double max = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));
            // Send calculated power to wheelsв
            if (max >= 1) {
                m1Drive.setPower(mochs*m1DrivePower *1/ max);
                m2Drive.setPower(mochs*m2DrivePower *1/ max);
                m3Drive.setPower(mochs*m3DrivePower *1/ max);
                m4Drive.setPower(mochs*m4DrivePower *1/ max);
            } else {
                m1Drive.setPower(mochs*m1DrivePower*1);
                m2Drive.setPower(mochs*m2DrivePower*1);
                m3Drive.setPower(mochs*m3DrivePower*1);
                m4Drive.setPower(mochs*m4DrivePower*1);
            }

//медленное движение
//----------------------------------------
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
            if(gamepad1.dpad_up){
                m1DrivePowerfordrivetofoundation11=0.3;
                m2DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m3DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m4DrivePowerfordrivetofoundation11=0.3;
            }else{
                m1DrivePowerfordrivetofoundation11=0;
                m2DrivePowerfordrivetofoundation11=0;
                m3DrivePowerfordrivetofoundation11=0;
                m4DrivePowerfordrivetofoundation11=0;
            }
            if(gamepad1.dpad_down){
                m1DrivePowerfordrivetofoundation1=-0.3;
                m2DrivePowerfordrivetofoundation1=0.3;
                m3DrivePowerfordrivetofoundation1=0.3;
                m4DrivePowerfordrivetofoundation1=-0.3;
            }else{
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
                m3DrivePowerfordrivetofoundation1=0;
                m4DrivePowerfordrivetofoundation1=0;
            }
            if(gamepad1.dpad_left){
                m1DrivePowerfordrivetofoundation=-0.33;
                m2DrivePowerfordrivetofoundation=-0.33;
                m3DrivePowerfordrivetofoundation=0.33;
                m4DrivePowerfordrivetofoundation=0.33;
            }else{
                m1DrivePowerfordrivetofoundation=0;
                m2DrivePowerfordrivetofoundation=0;
                m3DrivePowerfordrivetofoundation=0;
                m4DrivePowerfordrivetofoundation=0;
            }
            if(gamepad1.dpad_right){
                m1DrivePowerfordrivetofoundation2=0.33;
                m2DrivePowerfordrivetofoundation2=0.33;
                m3DrivePowerfordrivetofoundation2=-0.33;
                m4DrivePowerfordrivetofoundation2=-0.33;
            }else{
                m1DrivePowerfordrivetofoundation2=0;
                m2DrivePowerfordrivetofoundation2=0;
                m3DrivePowerfordrivetofoundation2=0;
                m4DrivePowerfordrivetofoundation2=0;
            }
//////////////////////////////////////////////////////////////////////

            if (zagrebalo != 0) {
                if (zagrebalo > 0) {
                    m6Intake.setPower(zagrebalo);
                } else {
                    m6Intake.setPower(zagrebalo);
                }
            } else {
                m6Intake.setPower(0);
            }


            /*if (caruselka_right > 0) {
                m7carousel.setPower(caruselka_right);
            } else {
                m7carousel.setPower(0);
            }

            if (caruselka_left > 0) {
                m7carousel.setPower(-1*caruselka_left);
            } else {
                m7carousel.setPower(0);
            }*/

            //if (touch.isPressed())

            if (zagrebalo > 0.1) {
                m8Val.setPower(1);
            }
            if (zagrebalo < -0.1) {
                m8Val.setPower(-1);
            }
            if (zagrebalo > -0.1){
                if (zagrebalo < 0.1) {
                    m8Val.setPower(0);
                }
            }
            if (zagrebalo < 0.1){
                if (zagrebalo > -0.1) {
                    m8Val.setPower(0);
                }
            }









            if (lift > 0.1) {
                s3Rotation.setPosition(0);
            }
            if (lift < -0.1) {
                s3Rotation.setPosition(0.4);
            }



            if (touch.isPressed() & lift > 0) {
                    m5Lift.setPower(0);
                    lift = 0;
            }

            //(touch.isPressed())
             if (lift !=0){
               m5Lift.setPower(lift);
            } else {m5Lift.setPower(0);}







            //while (!touchsensor.isPressed()) {
            //    telemetry.addData("isPressed",String.valueOf(touchsensor.isPressed()));
            //}





            /*if (gamepad2.y){
                    new ElapsedTime();
                    m5Lift.setPower(1);
                    s3Rotation.setPosition(0.5);
                    if (time > 1200) {
                        m5Lift.setPower(0);
                        s3Rotation.setPosition(1);
                    }

            }*/

            /*if (gamepad2.b){
                m5Lift.setPower(-1);
                s3Rotation.setPosition(0);
                if (!touchsensor.isPressed()) {
                    m5Lift.setPower(0);
                }
            }*/


            if (gamepad2.y) {
                //ElapsedTime lifttime = new ElapsedTime(Resolution.SECONDS);
                if (touch.isPressed()) {
                    lifttime.reset();
                } else {}

                //lifttime.getResolution();




                /*m5Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                m5Lift.setTargetPosition(-3500);

                // Switch to RUN_TO_POSITION mode

                m5Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                m5Lift.setVelocity(10000);

                while (m5Lift.isBusy()) {
                    //wait
                }
                m5Lift.setVelocity(0);
                m5Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
            }

            if (lifttime.milliseconds() < 1300) {
                m5Lift.setPower(-1);
                s3Rotation.setPosition(0.3);
            }

            //m5Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //m5Lift.setTargetPosition(1000);
            //m5Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //s3Rotation.setPosition(0.5);
            //m5Lift.setVelocity(LiftVelocity);



            if (gamepad1.b) {
                m1Drive.setPower(0.6);
                m2Drive.setPower(-0.62);
                m3Drive.setPower(0.6);
                m4Drive.setPower(-0.62);
                m5Lift.setPower(1);
                sleep (1200);
                m1Drive.setPower(0); m2Drive.setPower(0); m3Drive.setPower(0); m4Drive.setPower(0);

                m5Lift.setPower(0);
                s3Rotation.setPosition(0.7);
                sleep(1500);
                s3Rotation.setPosition(0);
                m5Lift.setPower(-1);
                sleep(1200);
                m5Lift.setPower(0);
            }
            if (gamepad1.x) {
                m1Drive.setPower(-0.6);
                m2Drive.setPower(0.62);
                m3Drive.setPower(-0.6);
                m4Drive.setPower(0.62);
                m5Lift.setPower(1);
                sleep (1200);
                m1Drive.setPower(0); m2Drive.setPower(0); m3Drive.setPower(0); m4Drive.setPower(0);

                m5Lift.setPower(0);
                s3Rotation.setPosition(0.7);
                sleep(1500);
                s3Rotation.setPosition(0);
                m5Lift.setPower(-1);
                sleep(1200);
                m5Lift.setPower(0);
            }

            //ковш
            if (gamepad2.right_bumper) {
                s3Rotation.setPosition(0.73);
            }
            if(gamepad2.left_bumper){
                s3Rotation.setPosition(0);
            }

            //автоматическое перекидывание ковша
            /*if (gamepad2.y) {
                m5Lift.setPower(1);
                sleep(1200);
                m5Lift.setPower(0);
                s3Rotation.setPosition(1);
                sleep(1000);
                s3Rotation.setPosition(0);
                m5Lift.setPower(-1);
                sleep(1200);
                m5Lift.setPower(0);
            }*/







            if (gamepad1.right_bumper) {
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);
                m7carousel.setPower(-0.3);
                sleep(1300);
                m7carousel.setPower(-1);
                sleep(400);
                m7carousel.setPower(0);
            }
            // Show the elapsed game time and wheel power.


            telemetry.addData("нажатие кнопки", touch.isPressed());


            telemetry.addData("Показания дальномера", distance.getDistance(DistanceUnit.CM));

            telemetry.addData("Status", "Lift Time: " + lifttime.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SErvo status", positionServo);
            telemetry.addData("SErvo position", s3Rotation.getPosition());
            telemetry.addData("Podiem position ", m5Lift.getCurrentPosition());

            telemetry.addData("angleofrotate", angles.firstAngle);


            //telemetry.addData("isPressed",String.valueOf(touchsensor.isPressed()));

            telemetry.addData("liftvelocity", m5Lift.getVelocity());
            telemetry.addData("liftposition", m5Lift.getCurrentPosition());


            // Loop while the motor is moving to the target
            /*while(m5Lift.isBusy()) {
                // Let the drive team see that we're waiting on the motor
                telemetry.addData("Status", "Waiting for the motor to reach its target");
                telemetry.update();
            }*/
             // The motor has reached its target position, and the program will continue
            
            
            
            //telemetry.addData("Distance left: ", DistanceSensor_left.getDistance(DistanceUnit.CM));
            //telemetry.addData("Distance right: ", DistanceSensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Motors", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m4DrivePower);
            telemetry.addData("Motors power for rotation", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePowerforrotation, m2DrivePowerforrotation, m3DrivePowerforrotation, m4DrivePowerforrotation);

            //telemetry.addData("current x position", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            //telemetry.addData("current y position", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            //telemetry.addData("current odometry angle", globalPositionUpdate.returnOrientation());
            //telemetry.addData("Vertical left encoder position", m1Drive.getCurrentPosition());
            //telemetry.addData("Vertical right encoder position", m2Drive.getCurrentPosition());
            //telemetry.addData("horizontal encoder position", m4Drive.getCurrentPosition());
            //  telemetry.addLine("Hardware variables successfully instantiated");
            telemetry.update();

        }


        /*
         * End of chassis related code.
         *
         * КОНЕЦ КОДА С ШАССИ
         */
















//----------------------------------------
//ручное передвижение ковша вверх-вниз
           /*{
                if(gamepad2.dpad_up){
                    m5Lift.setPower(1);
                }else if(gamepad2.dpad_down){
                    m5Lift.setPower(-1);
                }else{
                    m5Lift.setPower(0);
                }
            }*/



    }


}



