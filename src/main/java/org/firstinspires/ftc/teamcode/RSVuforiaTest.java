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

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RS: Vuforiatestauto", group = "Claire")



public class RSVuforiaTest extends RSLinearOpMode
{
    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    private int blue(int pixel) {
        return pixel & 0xff;
    }

    private int highestColor(int red, int green, int blue) {
        int[] color = {red,green,blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }


    public int calculateMiddle(Bitmap bmp)
    {
        int middle = 0;
        if (bmp != null) {
            int redValue = 0;
            int blueValue = 0;
            int greenValue = 0;


            int blueCounter = 0;
            int redCounter = 0;
            int blueEndCounter = 0;
            int redEndCounter = 0;
            int blueTarget = 100;
            int redTarget = 120;
            int blueStart = 0;
            int redStart = 0;
            int blueEnd = 0;
            int redEnd = 0;
            int x = bmp.getWidth()/2;
            for (int y = 0 ; y < bmp.getHeight(); y++) {
                int pixel = bmp.getPixel(x, y);
                redValue = red(pixel);
                blueValue = blue(pixel);

                if(y%10 == 0)
                {
                    Log.d("red " , Integer.toString(redValue));
                    Log.d("blue " , Integer.toString(blueValue));
                }


                if(blueCounter <= 5 && redCounter <= 5)
                {
                    if(redValue >= redTarget && blueValue<= blueTarget)
                    {
                        blueCounter = 0;
                        redCounter++;
                        if(redCounter == 5)
                        {
                            redStart = y - 4;
                        }
                    }
                    else if(blueValue >= blueTarget && redValue <= redTarget)
                    {
                        redCounter = 0;
                        blueCounter++;
                        if(blueCounter == 5)
                        {
                            blueStart = y - 4;
                        }
                    }
                }
                if(blueCounter >= 5)
                {
                    if(blueValue < blueTarget || redValue > redTarget)
                    {
                        blueEndCounter++;
                        if(blueEndCounter == 5)
                        {
                            blueCounter = 0;
                            blueEnd = y - 5;
                        }
                    }
                    else
                    {
                        blueEndCounter = 0;
                    }
                    if(y == bmp.getHeight() - 1)
                    {
                        blueEnd = y;
                    }
                }
                if(redCounter >= 5 )
                {
                    if(redValue < redTarget || blueValue > blueTarget)
                    {
                        redEndCounter++;
                        if(redEndCounter == 5) {
                            redCounter = 0;
                            redEnd = y - 5;
                        }
                    }
                    else
                    {
                        redEndCounter = 0;
                    }
                    if(y == bmp.getHeight() - 1)
                    {
                        redEnd = y;
                    }
                }


            }

            //flips y axis becuase y axis starts from "right" side of the image becuase it is stored sideways
            int tempBlueStart = blueStart;
            int tempRedStart = redStart;
            blueStart = bmp.getWidth()-blueEnd;
            blueEnd = bmp.getWidth()-tempBlueStart;
            redStart = bmp.getWidth()-redEnd;
            redEnd = bmp.getWidth()-tempRedStart;

            if((redStart < blueStart) && (redEnd > 0))
            {
                middle = redEnd + ((blueStart - redEnd)/2);
            }
            else if((blueStart < redStart) && (blueEnd > 0))
            {
                middle = blueEnd + ((redStart - blueEnd)/2);
            }
//            int color = highestColor(redValue, greenValue, blueValue);
//            String colorString = "";
//            switch (color) {
//                case 0:
//                    colorString = "RED";
//                    break;
//                case 1:
//                    colorString = "GREEN";
//                    break;
//                case 2:
//                    colorString = "BLUE";
//            }



            telemetry.addData("1Red Start:", " Red Start: " + redStart);
            telemetry.addData("2Red End:", " Red End: " + redEnd);
            telemetry.addData("3Blue Start", "Blue Start: " + blueStart);
            telemetry.addData("4Blue End", "Blue End: " + blueEnd);
            telemetry.addData("Middle", "Middle: " + middle);
            telemetry.update();



        }
//        telemetry.addData("Looped","Looped " + Integer.toString(looped) + " times");

//        Log.d("Looped:", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Looped " + looped);
//        Log.d("DEBUG:",data);
        return middle;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        Bitmap bitmap;

     //   InitNullHardware();
        InitHardware();
        //  robot.setMyAlliance(RSRobot.Alliance.BLUE);
        robot.stallDetectionOn = false;
        robot.InitVuforia();
        waitForStart();
            telemetry.addData("Test", "HELLO!");
            telemetry.update();


int i = 0;
        while (opModeIsActive())
        {
            i++;
            bitmap = robot.getVuforiaImage();


            telemetry.addData("RedValue", robot.getColorSensorRedValue());
            telemetry.addData("BlueValue", robot.getColorSensorBlueValue());
            telemetry.update();


         //   calculateMiddle(bitmap);
           sleep(100);

        }
    }
}
