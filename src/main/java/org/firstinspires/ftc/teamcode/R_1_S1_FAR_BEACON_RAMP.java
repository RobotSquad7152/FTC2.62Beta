package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Calendar;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: R_1_S1_FAR_BEACON_RAMP", group = "Claire")


public class R_1_S1_FAR_BEACON_RAMP extends RSLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        long targetrobotxpos;
        long targetrobotypos;
        long currentTime;
        long initialTime;

        InitHardware();
        robot.InitVuforia();
        //  robot.setMyAlliance(RSRobot.Alliance.RED);
        robot.stallDetectionOn = false;
        robot.DeactivateVuforia();
        waitForStart();
        initialTime = Calendar.getInstance().getTimeInMillis();
        RSRobot.RobotPos robotPos;

        robot.DriveBackward(.3, 25);
        robot.LaunchBall(1, robot.autoLaunchPow);

        int stepnum = 2;
        double[] power = new double[stepnum];
        long[] distance = new long[stepnum];
        double[] direction = new double[stepnum];
        long[] heading = new long[stepnum];


        //step 0
        power[0] = .45;
        distance[0] = 190;
        direction[0] = robot.backward;
        heading[0] = -45;

        //step 1
        power[1] = .45;
        distance[1] = 130;
        direction[1] = robot.backward;
        heading[1] = 45;


        robot.MultiDriveForwardToHeading(power, distance, direction, heading);

        robot.SpinLeft(.3, 75);

        currentTime = Calendar.getInstance().getTimeInMillis();

        sleep(10500-(currentTime-initialTime));
        robot.DriveForward(.4,40);
        robot.ActivateVuforia();
        do
        {
            robotPos = robot.GetVuforiaLocation();
        } while (robotPos.goodPos == false);

        Log.d("@@@@@@@@@after spin", "");
        telemetry.addData("RobotX ", robotPos.robotX);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
        //robotY = 1350 is perfect wall positon

        Log.d("@@@@Phone model:", Build.MODEL);
        Log.d("@@@@Phone make:", Build.MANUFACTURER);

        targetrobotxpos = toolsLeftTargetX;
        if (robot.isZTEphone())
            targetrobotypos = zteTargetY;
        else
            targetrobotypos = motoTargetY;
        Log.d("@@@@@@@@@first scan", "");
        Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" + robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" + robotPos.robotBearing);

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
            robot.DriveLeft(0.7, Math.abs((long) ((targetrobotxpos - robotPos.robotX) / 10)));
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
            robot.DriveRight(0.7, Math.abs((long) ((robotPos.robotX - targetrobotxpos) / 10)));
        }
        Log.d("@@@@@@@@@after drv left", "");


        do
        {
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
        } while (robotPos.goodPos == false);

        long distanceFromScan;
        distanceFromScan = (targetrobotypos - (long) robotPos.robotY) / 10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" + robotPos.robotX);
        Log.d("@@@VU distanceFromScan ", "" + distanceFromScan);

        telemetry.addData("Heading ", robotPos.robotBearing);
        Log.d("@@@@@@VU Heading ", "" + robotPos.robotBearing);


        robot.DriveBackwardToHeading(.2, distanceFromScan, (long) robotPos.robotBearing - 90);

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

        robot.scanRightPressBeacon(.3, "RED");







        robot.DriveForward(.45, 10);

        if (robot.wrongColorFirst) //blue is first
            robot.DriveRight(.6, 115);
        else
            robot.DriveRight(.6, 80);

        telemetry.addData("Blue first? ", robot.wrongColorFirst);
        telemetry.update();

        robot.stallDetectionOn = true;

        robot.DriveBackward(.4,50);

        robot.stallDetectionOn = false;

        robot.DriveForward(.3,10);
        /*robot.ActivateVuforia();
        do
        {
            robotPos = robot.GetVuforiaLocation();
        } while (robotPos.goodPos == false);

        Log.d("@@@@@@@@@after spin", "");
        telemetry.addData("RobotX ", robotPos.robotX);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
        //robotY = 1350 is perfect wall positon

        if (robot.isZTEphone())
            targetrobotypos = zteTargetY;
        else
            targetrobotypos = motoTargetY;
        targetrobotxpos = gearsRightTargetX;
        Log.d("@@@@@@@@@first scan", "");
        Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
        Log.d("@@@@@VU second robotX ", "" + robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" + robotPos.robotBearing);

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
            robot.DriveLeft(0.7, Math.abs((long) ((targetrobotxpos - robotPos.robotX) / 10)));
            *//*
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


            }*//*
            // robot.StopDrive();
        }
        else if (robotPos.robotX > targetrobotxpos)
        {
            robot.DriveRight(0.7, Math.abs((long) ((robotPos.robotX - targetrobotxpos) / 10)));
        }
        Log.d("@@@@@@@@@after drv left", "");


        do
        {
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
        } while (robotPos.goodPos == false);


        distanceFromScan = (targetrobotypos - (long) robotPos.robotY) / 10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" + robotPos.robotX);
        Log.d("@@@VU distanceFromScan ", "" + distanceFromScan);

        telemetry.addData("Heading ", robotPos.robotBearing);
        Log.d("@@@@@@VU Heading ", "" + robotPos.robotBearing);


        robot.DriveBackwardToHeading(.3, distanceFromScan, (long) robotPos.robotBearing - 90);*/

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

        robot.scanLeftPressBeacon(.3, "RED",3000);
        robot.DriveForwardToHeading(.7,200,100);
        robot.DriveForward(.7,90);
    }
}



