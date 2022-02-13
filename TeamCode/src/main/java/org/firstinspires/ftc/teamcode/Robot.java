/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class Robot extends LinearOpMode {
    protected static final int LED_CHANNEL = 5;
    protected DcMotor m1Drive = null;
    protected DcMotor m2Drive = null;
    protected DcMotor m3Drive = null;
    protected DcMotor m4Drive = null;
    protected Servo s1TopClaw = null;
    protected Servo s4Kicker = null;
    protected Servo s3Rotation = null;
    protected Servo s5Shovel = null;
    protected Servo s6RelicClaw = null;
    protected DcMotor m6Intake = null;
    protected DcMotor m5Lift = null;
    protected DcMotor m7carousel=null;
    protected DcMotor m8Val=null;

    protected Double shininessCoefficient = 1.8;

    protected float hsvValues[] = {0F, 0F, 0F};
    private String log = "";



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








    protected void lvl1(){
        m5Lift.setPower(1);
        sleep(400);
        m5Lift.setPower(0);
        s3Rotation.setPosition(1);
        sleep(1500);
        s3Rotation.setPosition(0);
        sleep(800);
        m5Lift.setPower(-0.8);
        sleep(400);
    }
    protected void lvl2(){
        m5Lift.setPower(1);
        sleep(750);
        m5Lift.setPower(0);
        s3Rotation.setPosition(1);
        sleep(1500);
        s3Rotation.setPosition(0);
        sleep(900);
        m5Lift.setPower(-1);
        if (touch.isPressed()) {
            m5Lift.setPower(0);
        }
    }
    protected void lvl3(){
        m5Lift.setPower(1);
        sleep(1350);
        m5Lift.setPower(0);
        s3Rotation.setPosition(1);
        sleep(1500);
        s3Rotation.setPosition(0);
        m5Lift.setPower(-1);sleep(1350);m5Lift.setPower(0);
    }
    protected void carousel() {
        m7carousel.setPower(-0.3);
        sleep(680);
        m7carousel.setPower(-0.9);
        sleep(850);
        m7carousel.setPower(0);
    }
    protected void carousell() {
        m7carousel.setPower(0.3);
        sleep(680);
        m7carousel.setPower(0.9);
        sleep(850);
        m7carousel.setPower(0);
    }


    protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        m1Drive.setPower(D1_power);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPower(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power +angles.firstAngle / 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power+ angles.firstAngle / 30);
        m4Drive.setPower(D4_power);
    }


    protected void setMotorsPowerforvard(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power );
        m2Drive.setPower(D2_power-(angle-angles.firstAngle )/ 24);
        m3Drive.setPower(D3_power);
        m4Drive.setPower(D4_power-(angle-angles.firstAngle) / 24);
    }

    protected void setMotorsPowerback(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 24);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 24);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPowerright(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 30);
        m4Drive.setPower(D4_power);
    }
    protected void setMotorsPowerleft(double D1_power, double D2_power, double D3_power, double D4_power, Orientation angles, BNO055IMU imu, double angle) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(D1_power - (angle-angles.firstAngle )/ 30);
        m2Drive.setPower(D2_power);
        m3Drive.setPower(D3_power - (angle-angles.firstAngle) / 30);
        m4Drive.setPower(D4_power);
    }

    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, double m5_power, double m6_power, long ms) {
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        m5Lift.setPower(m5_power);
        m6Intake.setPower(m6_power);

        sleep(ms);
        chassisStopMovement();
    }
    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, long ms, Orientation angles, BNO055IMU imu) {
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        sleep(ms);
        chassisStopMovement();
    }


    protected void setMotorsPowerTimed(double m1_power, double m2_power, double m3_power, double m4_power, long ms) {
        m1Drive.setPower(m1_power);
        m2Drive.setPower(m2_power);
        m3Drive.setPower(m3_power);
        m4Drive.setPower(m4_power);
        sleep(ms);
        chassisStopMovement();
    }
    protected void chassisStopMovement() {
        m1Drive.setPower(0);
        m2Drive.setPower(0);
        m3Drive.setPower(0);
        m4Drive.setPower(0);
        m5Lift.setPower(0);
        m6Intake.setPower(0);
    }
    DistanceSensor distance;
    TouchSensor touch;

    protected void initHW(HardwareMap hardwMap) throws RuntimeException {
        m1Drive = hardwMap.get(DcMotor.class, "m1 drive");
        m2Drive = hardwMap.get(DcMotor.class, "m2 drive");
        m3Drive = hardwMap.get(DcMotor.class, "m3 drive");
        m4Drive = hardwMap.get(DcMotor.class, "m4 drive");
        s1TopClaw = hardwMap.get(Servo.class, "s1 top claw");
        s4Kicker = hardwMap.get(Servo.class, "s4 kick");

        s3Rotation = hardwMap.get(Servo.class, "s3 rotation");
        s5Shovel = hardwMap.get(Servo.class, "s5 shovel");
        s6RelicClaw = hardwMap.get(Servo.class, "s6 relic claw");
        m6Intake = hardwMap.get(DcMotor.class, "m6 intake");

        m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        m7carousel = hardwareMap.get(DcMotor.class, "m7 rul");
        m8Val = hardwareMap.get(DcMotor.class, "m8 Val");

        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        m1Drive.setDirection(DcMotor.Direction.FORWARD);
        m2Drive.setDirection(DcMotor.Direction.FORWARD);
        m3Drive.setDirection(DcMotor.Direction.FORWARD);
        m4Drive.setDirection(DcMotor.Direction.FORWARD);
        //m1Drive.setDirection(DcMotor.Direction.REVERSE);
        //m2Drive.setDirection(DcMotor.Direction.REVERSE);
        //m3Drive.setDirection(DcMotor.Direction.REVERSE);
        //m4Drive.setDirection(DcMotor.Direction.REVERSE);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m5Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m5Lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m6Intake.setDirection(DcMotor.Direction.FORWARD);
        m8Val.setDirection(DcMotor.Direction.FORWARD);
        m7carousel.setDirection(DcMotor.Direction.FORWARD);
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m8Val.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}