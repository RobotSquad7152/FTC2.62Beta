package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: B_1_S1_FAR_BEACON", group = "Claire")


public class B_1_S1_FAR_BEACON extends RSLinearOpMode
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
        robot.DeactivateVuforia();
        waitForStart();

        RSRobot.RobotPos robotPos;

        robot.DriveBackward(.4, 25);
        robot.LaunchBall(1, robot.autoLaunchPow);

        int stepnum = 2;
        double[] power = new double[stepnum];
        long[] distance = new long[stepnum];
        double[] direction = new double[stepnum];
        long[] heading = new long[stepnum];


        //step 0
        power[0] = .45;
        distance[0] = 115;
        direction[0] = robot.backward;
        heading[0] = 45;

        //step 1
        power[1] = .45;
        distance[1] = 190;
        direction[1] = robot.backward;
        heading[1] = -45;


        robot.MultiDriveForwardToHeading(power, distance, direction, heading);

        robot.SpinRight(.3, 85);
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

        targetrobotxpos = legosRightTargetX;
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
            robot.SpinLeft(.9, (long) (90-robotPos.robotBearing));
        }
        else if (robotPos.robotBearing > 90)
        {
            robot.SpinRight(.9, (long) (robotPos.robotBearing-90));
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


        robot.DriveBackwardToHeading(.3, distanceFromScan, (long) robotPos.robotBearing - 90);

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

        robot.scanLeftPressBeacon(.3, "BLUE");


        robot.DriveForward(.45, 25);

        if (robot.wrongColorFirst) //red is first
            robot.DriveLeft(.6, 145);
        else
            robot.DriveLeft(.6, 120);

        telemetry.addData("Red first? ", robot.wrongColorFirst);
        telemetry.update();


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

        if (robot.isZTEphone())
            targetrobotypos = zteTargetY;
        else
            targetrobotypos = motoTargetY;
        targetrobotxpos = wheelsLeftTargetX;
        Log.d("@@@@@@@@@first scan", "");
        Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" + robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" + robotPos.robotBearing);

        if (robotPos.robotBearing < 90)
        {
            robot.SpinLeft(.9, (long) (90-robotPos.robotBearing));
        }
        else if (robotPos.robotBearing > 90)
        {
            robot.SpinRight(.9, (long) (robotPos.robotBearing-90));
        }


        if (robotPos.robotX < targetrobotxpos)
        {
            robot.DriveLeft(0.7, (long) ((targetrobotxpos - robotPos.robotX) / 10));
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
            robot.DriveRight(0.7, (long) ((robotPos.robotX - targetrobotxpos) / 10));
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


        robot.DriveBackwardToHeading(.3, distanceFromScan, (long) robotPos.robotBearing - 90);

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

    }
}
