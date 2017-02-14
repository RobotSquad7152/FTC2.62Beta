package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: B_3_BC", group = "Claire")

public class B_3_BC extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        InitHardware();
        //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = false;
        waitForStart();
        RSRobot.RobotPos robotPos;
        robot.DriveBackward(.4,50);
        robot.LaunchBall(2,robot.autoLaunchPow);
        robot.DriveBackward(.4,90);
        robot.SpinRight(.4,45);
        robot.SpinLeft(.4,45);
        robot.DriveBackward(.4,20);


    }
}
