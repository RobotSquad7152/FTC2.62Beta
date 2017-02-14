package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: B_1_BEACON_2", group = "Claire")


public class B_1_BEACON_2 extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        long targetrobotxpos;
        long targetrobotypos;

        InitHardware();
        robot.InitVuforia();
        //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = false;
        waitForStart();

        RSRobot.RobotPos robotPos;


        robot.DriveBackward(.4,25);
        robot.LaunchBall(2, robot.autoLaunchPow);

        int stepnum = 2;
        double[] power = new double[stepnum];
        long[] distance = new long[stepnum];
        double[] direction = new double[stepnum];
        long[] heading = new long[stepnum];


        //step 0
        power[0] = .45;
        distance[0] = 35;
        direction[0] = robot.backward;
        heading[0] = 90;




        //step 1
        power[1] = .45;
        distance[1] = 70;
        direction[1] = robot.right;
        heading[1] = 0;
/*
        //step 2

        power[2] = .9;
        distance[2] = 5;
        direction[2] = robot.backward;
        heading[2] = 45;
        */

        robot.MultiDriveForwardToHeading(power, distance, direction, heading);


        //robot.servoColor.setPosition(robot.servoColorScan);


        /*
        robot.DriveBackward(.4,10);
        robot.SpinRight(.4,45);
        robot.servoColor.setPosition(robot.servoColorScan);
        robot.DriveBackward(.4,105);
        robot.SpinRight(.4,40);
        robot.DriveForward(.4, 10);
        */

     //  sleep (500);



        do{
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
            telemetry.update();
           // idle();
        }while( robotPos.goodPos == false);

        targetrobotypos = 1750;
        targetrobotxpos = 250;
        Log.d("@@@@@@@@@first scan", "");
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);

        if (robotPos.robotBearing < 90)
        {
            robot.SpinLeft(.9, (long) (Math.abs(robotPos.robotBearing) - 90));
        }
        else if (robotPos.robotBearing > 90)
        {
            robot.SpinRight(.9, (long) (90 - (Math.abs(robotPos.robotBearing))));
        }

        if (robotPos.robotX < targetrobotxpos)
        {
            robot.DriveLeft(0.7,(long)((targetrobotxpos-robotPos.robotX)/10));
        }
        else if (robotPos.robotX > targetrobotxpos)
        {
            robot.DriveRight(0.7,(long)((robotPos.robotX-targetrobotxpos)/10));

        }
        Log.d("@@@@@@@@@after drv left", "");

        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);




/*        do
        {
            robotPos = robot.GetVuforiaLocation();
        } while( robotPos.goodPos == false);*/

        //Log.d("@@@@@@@@@after spin", "");
        telemetry.addData("RobotY ", robotPos.robotY);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
    //robotY = 1350 is perfect wall positon

    //    sleep(300);


        do{
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
            telemetry.update();
            // idle();
        }while( robotPos.goodPos == false);



        robot.DeactivateVuforia();

        long distanceFromScan = (targetrobotypos - (long) robotPos.robotY)/10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@VU distanceFromScan ", "" +distanceFromScan);

        telemetry.addData("Heading ", robotPos.robotBearing);
        Log.d("@@@@@@VU Heading ", "" +robotPos.robotBearing);


        robot.DriveBackwardToHeading(.3, distanceFromScan,(long)robotPos.robotBearing - 90);
        robot.scanRightPressBeacon(.3, "BLUE");




       /* int stepnum2 = 2;
        double[] power2 = new double[stepnum2];
        long[] distance2 = new long[stepnum2];
        double[] direction2 = new double[stepnum2];
        long[] heading2 = new long[stepnum2];

        //step 0
        power2[0] = .45;
        distance2[0] = 15;
        direction2[0] = robot.forward;
        heading2[0] = 0;

        //step 1
        power2[1] = .8;
        distance2[1] = 100;
        direction2[1] = robot.right;
        heading2[1] = 0;*/

        //robot.MultiDriveForwardToHeading(power2, distance2, direction2, heading2);

        robot.DriveForward(.45, 25);

        if (robot.wrongColorFirst) //red is first
            robot.DriveRight(.6, 145);
        else
            robot.DriveRight(.6, 120);

        telemetry.addData("Red first? ", robot.wrongColorFirst);
        telemetry.update();


        robot.ActivateVuforia();
        do
        {
            robotPos = robot.GetVuforiaLocation();
        } while( robotPos.goodPos == false);

        Log.d("@@@@@@@@@after spin", "");
        telemetry.addData("RobotX ", robotPos.robotX);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
        //robotY = 1350 is perfect wall positon

        targetrobotxpos = -800;
        targetrobotypos = 1720;
        Log.d("@@@@@@@@@first scan", "");
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);

        if (robotPos.robotBearing < 90)
        {
            robot.SpinLeft(.9, (long) (Math.abs(robotPos.robotBearing) - 90));
        }
        else if (robotPos.robotBearing > 90)
        {
            robot.SpinRight(.9, (long) (90 - (Math.abs(robotPos.robotBearing))));
        }


        if (robotPos.robotX < targetrobotxpos)
        {
            robot.DriveLeft(0.7,(long)((targetrobotxpos-robotPos.robotX)/10));
            /*
            while (robotPos.robotX < targetrobotxpos)
            {
                robotPos = robot.GetVuforiaLocation();
                robot.DriveRightNoRamp(0.4);
                telemetry.addData("Visible", robotPos.goodPos);
                telemetry.addData("RobotX ", robotPos.robotX);
                telemetry.addData("Heading ", robotPos.robotBearing);
                telemetry.update();
                Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
                Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
                Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);


            }*/
           // robot.StopDrive();
        }
        else if (robotPos.robotX > targetrobotxpos)
        {
            robot.DriveRight(0.7,(long)((robotPos.robotX-targetrobotxpos)/10));
        }
        Log.d("@@@@@@@@@after drv left", "");


        do
        {
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
        } while( robotPos.goodPos == false);


        distanceFromScan = (targetrobotypos - (long) robotPos.robotY)/10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@VU distanceFromScan ", "" +distanceFromScan);

        telemetry.addData("Heading ", robotPos.robotBearing);
        Log.d("@@@@@@VU Heading ", "" +robotPos.robotBearing);


        robot.DriveBackwardToHeading(.3, distanceFromScan,(long)robotPos.robotBearing - 90);

/*        do
        {
            robotPos = robot.GetVuforiaLocation();
        } while( robotPos.goodPos == false);


        if (robotPos.robotX < targetrobotxpos)
        {
            robot.DriveLeft(0.7,(long)((targetrobotxpos-robotPos.robotX)/10));
        }
        else if (robotPos.robotX > targetrobotxpos)
        {
            robot.DriveRight(0.7,(long)((robotPos.robotX-targetrobotxpos)/10));

        }

        robot.DeactivateVuforia();*/

        robot.scanRightPressBeacon(.3, "BLUE");



/*
        robot.DriveForward(1, 80);

        robot.SpinLeft(.8, 45);

        robot.SpinRight(.8, 45);

        robot.DriveForward(1, 20);
        */


    }
}
