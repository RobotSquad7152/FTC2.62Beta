package org.firstinspires.ftc.teamcode;/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.util.Log;

@Autonomous(name = "RS: testauto", group = "Claire")



public class RSTestAuto extends RSLinearOpMode
{
    double targetrobotxpos;
    double targetrobotypos;
    double targetdistance;
    double turnangle;
    @Override
    public void runOpMode() throws InterruptedException
    {

        InitHardware();

      //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = true;
        robot.InitVuforia();
        Log.d("@@@@@@@@@ ", "after Vuforia init");

        waitForStart();
        robot.DriveBackward(.5,70);
        robot.DriveForward(.5,20);

        sleep(2000);

        /*
        Log.d("@@@@@@@@@","ready to spin");
        robot.SpinRight(.7, 90);
        */



/*        int stepnum = 1;
        double[] power = new double[stepnum];
        long[] distance = new long[stepnum];
        double[] direction = new double[stepnum];
        long[] heading = new long[stepnum];


        //step 0
        power[0] = .45;
        distance[0] = 100;
        direction[0] = robot.right;
        heading[0] = 0;*/



/*
        //step 1
        power[1] = .9;
        distance[1] = 70;
        direction[1] = robot.right;
        heading[1] = 0;

        */
/*
        //step 2

        power[2] = .9;
        distance[2] = 5;
        direction[2] = robot.backward;
        heading[2] = 45;
        */

        //robot.MultiDriveForwardToHeading(power, distance, direction, heading);


   /*     power[1] = .5;
        distance[1] = 150;
        direction[1] = robot.forward;
        heading[1] = 90;

        power[2] = .8;
        distance[2] = 105;
        direction[2] = robot.left;
        heading[2] = -30;

        power[3] = .7;
        distance[3] = 90;
        direction[3] = robot.backward;
        heading[3] = -45;

        power[4] = .7;
        distance[4] = 140;
        direction[4] = robot.backward;
        heading[4] = -45; */

        //robot.MultiDriveForwardToHeading(power, distance, direction, heading);
       //RSRobot.RobotPos robotPos;
      //  robot.servoColor.setPosition(robot.servoColorScan);

        /*while(opModeIsActive())
        {
            *//*telemetry.addData("blue " ,robot.color.blue());
            telemetry.addData("red " ,robot.color.red());*//*

            robotPos = robot.GetVuforiaLocation();
            telemetry.addData("RobotY ", robotPos.robotY);
            telemetry.addData("RobotX ", robotPos.robotX);
            telemetry.addData("Heading ", robotPos.robotBearing);
            telemetry.addData("goodPos ", robotPos.goodPos);
            telemetry.update();
            //robotY = 1380 is perfect wall positon
            long distanceFromScan = (1800 - (long) robotPos.robotY - 450) / 10; // must go from mm to cm
            Log.d("@@@@@@@@@VU robotY ", "" + robotPos.robotY);
            Log.d("@@@@@@@@@ robotX","" + robotPos.robotX);
            Log.d("@@VU disatancefromscan ", "" + distanceFromScan);
        }*/
        //robot.DriveBackward(.3, distanceFromScan);

          //  idle();
     //   }

      /*
       do{
           robotPos = robot.GetVuforiaLocation();
           telemetry.addData("Visible", robotPos.goodPos);
           telemetry.addData("RobotX ", robotPos.robotX);
           telemetry.addData("RobotY ", robotPos.robotY);
           telemetry.addData("Heading ", robotPos.robotBearing);
           telemetry.update();
           idle();
    }while( robotPos.goodPos == false);

        targetrobotypos = 1080;
        targetrobotxpos = 300;



     //   turnangle = Math.toDegrees(Math.atan((targetrobotypos-robotPos.robotY)/(targetrobotxpos-robotPos.robotX)));

     //   targetdistance = Math.sqrt(Math.pow(targetrobotypos-robotPos.robotY,2)+Math.pow(targetrobotxpos-robotPos.robotX,2))/10;

       Log.d("robot y pos", Double.toString(robotPos.robotY));
        Log.d("robot x pos", Double.toString(robotPos.robotX));

        */

        /*
        Log.d("angle ",  Double.toString(turnangle));
        Log.d("distance ", Double.toString(targetdistance));

        if(turnangle > 0)
        {
            robot.SpinLeft(.4, Math.abs((long)turnangle));
        }
        else if(turnangle< 0)
        {
            robot.SpinRight(.4,Math.abs((long)turnangle));
        }

        robot.DriveBackward(.75, (long)targetdistance);

        robotPos.robotBearing += turnangle;

        turnangle = 90 - robotPos.robotBearing;

        if(turnangle > 0)
        {
            robot.SpinLeft(.75, Math.abs((long)turnangle));
        }
        else if(turnangle< 0)
        {
            robot.SpinRight(.75,Math.abs((long)turnangle));
        }

        */




        /*while(true)
        {
           robotPos = robot.GetVuforiaLocation();
            if (robotPos.goodPos)
            {
                telemetry.addData("RobotX ", robotPos.robotX);
                telemetry.addData("RobotY ", robotPos.robotY);
                telemetry.addData("Heading ", robotPos.robotBearing);
                telemetry.update();
            }
            else
            {
                telemetry.addData("Pos", "Unknown");
            }*/
        //}




  //      robot.DriveForward(.5,70);
  //      robot.DriveRight(.5,70);
         //robot.DriveLeft(.8,100);

        //robot.DeliverClimber();

//        robot.stallDetectionOn = false;
//        robot.DriveToUSDistance(20);
   //     while (true)
        //    telemetry.addData("Ultrasonic value ", robot.readUltrasonic());

        /*
        robot.DriveForward(.5,7);
        robot.SpinLeft(.7, 45);
        robot.DriveForwardToTape(.2, 100, RSRobot.Alliance.WHITE);
        robot.DriveBackward(.5, 15);
        robot.DeliverClimberRight();
        */

        //robot.DeliverClimberRight();
        // robot.startHarvester();

  //      robot.DriveForward(.7, 200)
//        for(int i = 0; i < 5; i++)
//        {
//            robot.SpinLeft(1, 90);
//        }
//
//        sleep(500);

       // robot.DriveForward(.5,50);

        //Log.d("RSTESTAUTOIIIIIIIIIII", "DONE!!!!!!!!!!!!!");


       //robot.SpinRight(.7, 360);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "About to spin");
//        robot.SpinRight(.7, 90);
//
//        Log.d("RSTESTAUTOIIIIIIIIIII", "Spun!");
//        sleep(500);

        //    robot.SpinRight(1, 90);
        //  sleep(1000);

        //   robot.SpinLeft(1,90);


//    //robot.DriveBackward(.1, 1000);
//      for( int i=0; i<3000; i++)
//      {
//          sleep(10);
//          telemetry.addData("curr heading ", robot.GetCurrentHeading());
//      }
//    robot.stopHarvester();


           /*  for (int count = 0; count < 8; count ++)
             {
                 robot.DriveBackward(.5, 100);
                 sleep(500);

                 robot.DriveForward(.5, 100);
                 sleep(500);
             }*/


//robot.CrazyDrive();

        // robot.DriveForwardWall(.3, 100, 10);





    }
}
