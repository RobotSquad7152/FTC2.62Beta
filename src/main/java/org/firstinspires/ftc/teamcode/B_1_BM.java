package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: B_1_BM", group = "Claire")

public class B_1_BM extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        InitHardware();
        //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = false;
        waitForStart();
        RSRobot.RobotPos robotPos;
        robot.DriveBackward(.4,25);
        robot.LaunchBall(2,robot.autoLaunchPow);
        robot.DriveBackward(.4,75);
        robot.SpinRight(.4,45);
        sleep(250);
        robot.SpinLeft(.4,50);
        robot.DriveBackward(.5,40);

    }
}
