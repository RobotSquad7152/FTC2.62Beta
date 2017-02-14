package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import java.util.Calendar;

/**
 * Created by Tony on 9/20/2015.
 */
public class GyroThread extends Thread
{


    double currentHeading;
    double calibratedGyroRotation;
    ModernRoboticsI2cGyro gyro;

    public GyroThread(ModernRoboticsI2cGyro g)
    {
        currentHeading = 0;
        gyro = g;
        calibratedGyroRotation = 0;


    }


    public double getCurrentHeading()
    {
        return currentHeading;
    }

    public void setCurrentHeading(double currentHeading)
    {
        this.currentHeading = currentHeading;
    }
    public void calibrategyro()
    {
        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            try
            {
                sleep(100);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }

    public void RScalibrategyro()
    {
        double totalGyroReadings = 0;

        //set the millisecond timer
        //    long millisStart = Calendar.getInstance().getTimeInMillis();


        // Take 1000 readings and average them out
        for (int i = 0; i < 500; i++)
        {
            // Wait until 3ms has passed
            //   while (Calendar.getInstance().getTimeInMillis() < millisStart + 3)
            //       try {
            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            //     } catch (InterruptedException e) {
            //          e.printStackTrace();
            //      }

            // millisStart = Calendar.getInstance().getTimeInMillis();
            totalGyroReadings += (double)(gyro.rawZ()/128);
       //     Log.d( "@@@@@Cal RawZ", String.valueOf(gyro.rawZ()/128));

        }
        calibratedGyroRotation = totalGyroReadings / 500;
    }

    public void run()
    {
        double scaleFactor = 2.72;
        long prevTime = Calendar.getInstance().getTimeInMillis();
        long currTime;
        do
        {

            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }

            currTime = Calendar.getInstance().getTimeInMillis();
            //calculate turn based on gyro reading divided by the time taken since the last reading (6ms)
            currentHeading = currentHeading + scaleFactor * (((double)(gyro.rawZ()/128) - calibratedGyroRotation) * ((double)(currTime-prevTime)/1000));
            prevTime = currTime;

          //  Log.d( "@@@@@RawZ", String.valueOf(gyro.rawZ()/128));
           // Log.d( "@@@@@@cal", String.valueOf(calibratedGyroRotation));

        } while (true);

    }
}
