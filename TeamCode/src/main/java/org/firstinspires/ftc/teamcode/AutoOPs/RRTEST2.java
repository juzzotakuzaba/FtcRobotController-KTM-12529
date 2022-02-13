package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RRTEST2 extends LinearOpMode {
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = drive.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = drive.trajectoryBuilder(startPose, true)
                .lineToConstantHeading(new Vector2d(-25, 34))
                .build();


        Trajectory secondd = drive.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(-25, 34, Math.toDegrees(-90)))
                .build();

        Trajectory second = drive.trajectoryBuilder(secondd.end(), Math.toDegrees(-90))
                .strafeTo(new Vector2d(-20, 35))
                //.strafeLeft(40)
                .build();

        Trajectory third = drive.trajectoryBuilder(second.end(), false)
                .forward(40)
                .build();




        waitForStart();
        {
            //drive.followTrajectory(firstt);
            //sleep(1000);
            drive.setMotorPowers(-0.3,-0.3,0.3,0.3);
            sleep(700);
            drive.setMotorPowers(0,0,0,0);
            drive.setMotorPowers(0.3,-0.3,-0.3,0.3);
            sleep(700);

        }
    }
}
