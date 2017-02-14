package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: B_1_BEACON", group = "Claire")


public class B_1_BEACON extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        double targetrobotxpos;
        double targetrobotypos;

        InitHardware();
        robot.InitVuforia();
        //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = false;
        waitForStart();

        RSRobot.RobotPos robotPos;


        robot.DriveBackward(.4,25);
        robot.LaunchBall(2, robot.autoLaunchPow);
        robot.DriveBackward(.4,10);
        robot.SpinRight(.4,45);
        //robot.servoColor.setPosition(robot.servoColorScan);
        robot.DriveBackward(.4,105);
        robot.SpinRight(.4,40);
        robot.DriveForward(.4, 10);

     //  sleep (500);

        do{
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
            telemetry.update();
            idle();
        }while( robotPos.goodPos == false);

        //targetrobotypos = 1080;
        targetrobotxpos = 250;
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);

        if (robotPos.robotX < targetrobotxpos)
        {
            robot.DriveLeft(0.7,(long)((targetrobotxpos-robotPos.robotX)/10));
    /*        while (robotPos.robotX < targetrobotxpos)
            {
                robotPos = robot.GetVuforiaLocation();
                robot.DriveRightNoRamp(1);
                telemetry.addData("Visible", robotPos.goodPos);
                telemetry.addData("RobotX ", robotPos.robotX);
                telemetry.addData("Heading ", robotPos.robotBearing);
                telemetry.update();
                Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
                Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
                Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);

            }
            robot.StopDrive();*/
        }

        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);


        if (robotPos.robotBearing < 90)
        {
            robot.SpinLeft(.7, (long) (Math.abs(robotPos.robotBearing) - 90));
        }
        else if (robotPos.robotBearing > 90)
        {
            robot.SpinRight(.7, (long) (90 - (Math.abs(robotPos.robotBearing))));
        }

        do
        {
            robotPos = robot.GetVuforiaLocation();
        }while( robotPos.goodPos == false);
        telemetry.addData("RobotY ", robotPos.robotY);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
    //robotY = 1350 is perfect wall positon
        float robotY1 = robotPos.robotY;
        Log.d("@@@@@@@@@VU robotY1 ", "" +robotY1);

    //    sleep(300);

        do
        {
            robotPos = robot.GetVuforiaLocation();
        }while( robotPos.goodPos == false);
        telemetry.addData("RobotY ", robotPos.robotY);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
        float averageRobotY = (robotY1 + robotPos.robotY) / 2;
        Log.d("@@@@@@@@@VU robotY2 ", "" +robotPos.robotY);




        long distanceFromScan = (1360 - (long) averageRobotY)/10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        robot.DriveBackwardToHeading(.3, distanceFromScan,(long)robotPos.robotBearing - 90);
        robot.scanRightPressBeacon(.4, "BLUE");



        robot.DriveForward(1, 80);

        robot.SpinLeft(.8, 45);

        robot.SpinRight(.8, 45);

        robot.DriveForward(1, 20);


    }
}
