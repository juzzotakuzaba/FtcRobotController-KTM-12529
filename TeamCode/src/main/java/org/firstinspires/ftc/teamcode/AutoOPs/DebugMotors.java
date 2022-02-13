/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/
package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;

@Autonomous(name= "DebugMotors", group="AutoOP")
public class DebugMotors extends Robot {
    private ElapsedTime runtime = new ElapsedTime();
    private int time = 3000; //One way rotation time (ms)

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision();

        int ShElementPosition;
        if((pipeline.position == EasyOpenCVVision.ShipPosition.LEFT)){
            ShElementPosition = 3;
        }
        if((pipeline.position == EasyOpenCVVision.ShipPosition.CENTER)){
            ShElementPosition = 2;
        }
        if((pipeline.position == EasyOpenCVVision.ShipPosition.NONE)){
            ShElementPosition = 1;
        }



        telemetry.addData("avg1", pipeline.getAnalysis() );
        telemetry.addData("avg2", pipeline.getAnalysis2() );
        telemetry.addLine("Webcam ready for start");
        telemetry.update();
        waitForStart();
        {

            telemetry.clear();
            runtime.reset();


            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            imu.initialize(parameters);
            double time1;
            double time2;
            time1=getRuntime();
            time2=getRuntime();
            while(opModeIsActive()&&!isStopRequested()) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //telemetry.addData("Right sensor", DistanceSensor_right.getDistance(DistanceUnit.CM));
                //telemetry.addData("Left sensor", DistanceSensor_left.getDistance(DistanceUnit.CM));
                //telemetry.addData("Forward sensor", DistanceSensor_forward.getDistance(DistanceUnit.CM));
                telemetry.addData("angle1", angles.firstAngle);
                telemetry.addData("angle2", angles.secondAngle);
                telemetry.addData("angle3", angles.thirdAngle);
                //telemetry.addData("avg1", pipeline.getAnalysis() );
                //telemetry.addData("avg2", pipeline.getAnalysis() );
                telemetry.addData("ShElementPosition", pipeline.getAnalysis() );
                telemetry.update();
            }
        }
    }
}
