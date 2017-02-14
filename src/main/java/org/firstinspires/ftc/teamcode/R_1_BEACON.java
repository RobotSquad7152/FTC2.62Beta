package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Tony on 11/10/2016.
 */
@Autonomous(name = "RS: R_1_BEACON", group = "Claire")


public class R_1_BEACON extends RSLinearOpMode
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
        robot.LaunchBall(2,robot.autoLaunchPow);
        robot.DriveBackward(.4,10);
        robot.SpinLeft(.4,45);
        //robot.servoColor.setPosition(robot.servoColorScan);
        robot.DriveBackward(.4,90);
        robot.SpinLeft(.4,50);
        //robot.DriveForward(.4, 10);

     //   sleep (500);

        do{
            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("Visible", robotPos.goodPos);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("Heading ", robotPos.robotBearing);
            telemetry.update();
            idle();
        }while( robotPos.goodPos == false);

        targetrobotypos = -370;
        //targetrobotxpos = 215;
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);
        Log.d("@@@@@@VU robotBearing ", "" +robotPos.robotBearing);
        if (robotPos.robotY > targetrobotypos)
        {
            robot.DriveRight(0.7,(long)(Math.abs(targetrobotypos-robotPos.robotY)/10));
            /*while (robotPos.robotY > targetrobotypos)
            {
                robotPos = robot.GetVuforiaLocation();
                robot.DriveRightNoRamp(.4);
                telemetry.addData("Visible", robotPos.goodPos);
                telemetry.addData("RobotY ", robotPos.robotY);
                telemetry.addData("Heading ", robotPos.robotBearing);
                telemetry.update();
            }
            robot.StopDrive();*/
        }

        /*if (robotPos.robotBearing < 180)
        {
            robot.SpinLeft(.7, (long) (Math.abs(robotPos.robotBearing))-180);
        }
        else if (robotPos.robotBearing > 180)
        {
            robot.SpinRight(.7, (long) (180-(Math.abs(robotPos.robotBearing))));
        }*/
        do
        {
            robotPos = robot.GetVuforiaLocation();
        }while( robotPos.goodPos == false);

        robotPos = robot.GetVuforiaLocation();
        telemetry.addData("RobotY ", robotPos.robotY);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();

        float robotX1 = robotPos.robotX;
        Log.d("@@@@@@@@@VU robotY1 ", "" +robotX1);

        do
        {
            robotPos = robot.GetVuforiaLocation();
        }while( robotPos.goodPos == false);
        telemetry.addData("RobotX ", robotPos.robotX);
        telemetry.addData("Heading ", robotPos.robotBearing);
        telemetry.update();
        float averageRobotX = (robotX1 + robotPos.robotX) / 2;
        Log.d("@@@@@@@@@VU robotX2 ", "" +robotPos.robotX);

        //long distanceFromScan = (1800 - (long) robotPos.robotX - 70 - 160)/10; // must go from mm to cm
        long distanceFromScan = (1375 - Math.abs((long) averageRobotX))/10; // must go from mm to cm // was originally 1330
        Log.d("@@@@@@@@@VU robotY ", "" +robotPos.robotY);
        Log.d("@@@@@@@@@VU robotX ", "" +robotPos.robotX);


        long newheading;
        if(robotPos.robotBearing<-90){
            newheading = (long) (robotPos.robotBearing+180);
        }else if(robotPos.robotBearing>90){
            newheading = (long) (robotPos.robotBearing-180);
        }else{
            newheading=0;
        }

        //add if longer than certain amount do it

        /*
        if (distanceFromScan <

         */
        robot.DriveBackwardToHeading(.3, distanceFromScan,newheading);
        robot.scanRightPressBeacon(.4, "RED");


        robot.DriveForward(1, 80);

        robot.SpinRight(.8, 45);

        robot.SpinLeft(.8, 45);

        robot.DriveForward(1, 20);


    }
}
