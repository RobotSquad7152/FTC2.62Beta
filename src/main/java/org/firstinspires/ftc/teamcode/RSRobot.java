/* testing git */
/* testing */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.concurrent.BlockingQueue;

import static java.lang.Math.abs;

//import javax.imageio.*;
//import java.awt.image.BufferedImage;

/**
 * This is NOT an opmode.
 * <p/>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 * <p/>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p/>
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 * <p/>
 * Note: the configuration of the servos is such that:
 * As the arm servo approaches 0, the arm position moves up (away from the floor).
 * As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class RSRobot
{
    /* Public OpMode members. */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorLaunchLeft = null;
    public DcMotor motorLaunchRight = null;
    public DcMotor motorIntake = null;
    public DcMotor motorIntake2 = null;
    public Servo servoSetter = null;
    public Servo servoClutch = null;
    public Servo servoCapRight = null;
    public Servo servoCapLeft = null;
    //public Servo servoColor = null;
    public ColorSensor color = null;
    //    public FtcI2cDeviceState colorSensorState;
    public ModernRoboticsI2cColorSensor colorSensor;
    private ModernRoboticsI2cGyro gyro;
    // public GyroSensor gyro = null;

    private GyroThread gyrothread;


    public boolean stallDetectionOn = false;
    private double targetHeading;
    RSLinearOpMode opMode;
    /* Local OpMode members. */
    HardwareMap hwMap = null;
    //defines drive wheel diameter -- a 2 inch sprocket this year
    final double wheeldiacm = 4.4 * 2.54; //CHANGED FROM 4 TO 4.4!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //defines drive wheel circumference
    final double wheelcirccm = wheeldiacm * 3.141592653589793;
    //defines how many teeth on gear attached to motor (no gearing this year)
    final double motorgearteeth = 3;
    //defines how many teeth on gear attached to wheel (no gearing this year)
    final double wheelgearteeth = 2;
    //encoder counts per rotation of the motor
    final double motorclicksperrotation = 1120;
    //calculates how far the robot will drive for each motor encoder click
    final double onemotorclick = ((motorgearteeth / wheelgearteeth) * wheelcirccm) / motorclicksperrotation;


    Boolean wrongColorFirst = false;
    Boolean beaconFound = false;


    public static final int forward = 1;
    public static final int backward = -1;
    public static final int right = 2;
    public static final int left = -2;

    public double servoSetterUpPos = 220.0 / 225.0;
    public double servoSetterRapidPos;
    public double servoSetterDownPos = 150.0 / 255.0;
    public double servoClutchEngagedPos = 150.0 / 255.0;
    public double servoClutchLockedPos = 220.0 / 225.0;
    //^^ winch axel will not move at all
    public double servoClutchDisengagePos = 190.0 / 225.0;
    //^^ winch axel spins freely

    public double servoCapRightInitPos = 245.0 / 255.0;
    public double servoCapRightHoldPos = 190.0 / 255.0;
    public double servoCapRightParallel = 150.0 / 255.0;
    public double servoCapLeftInitPos = 40.0 / 255.0;
    public double servoCapLeftHoldPos = 150.0 / 255.0;
    public double servoCapLeftParallel = 190.0 / 255.0;

    double autoLaunchPow = .45;




    /*public double servoColorIn = .99;
    public double servoColorScan = .58;
    public double servoColorPush = .45;*/

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    Bitmap bitmap;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;
    VuforiaTrackables fieldGraphics;


    public class RobotPos
    {
        public float robotX = 0;
        public float robotY = 0;
        public float robotBearing = 0;
        public boolean goodPos = true;
    }

    RobotPos lastRobotPos;

    /* Constructor */
    public RSRobot()
    {
    }

    public void initNull(HardwareMap ahwMap)
    {
        // save refeMrence to HW Map
        hwMap = ahwMap;

        lastRobotPos = new RobotPos();

        //gyro = hwMMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);


    }

    /* InitializeGyro standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // save reference to HW Map
        hwMap = ahwMap;

        lastRobotPos = new RobotPos();

        // Define and Initialize Motors
        motorIntake = hwMap.dcMotor.get("motor_intake_1"); //
        motorIntake2 = hwMap.dcMotor.get("motor_intake_2"); //
        motorBackRight = hwMap.dcMotor.get("motor_back_right"); //
        motorBackLeft = hwMap.dcMotor.get("motor_back_left"); //
        motorFrontLeft = hwMap.dcMotor.get("motor_front_left");
        motorFrontRight = hwMap.dcMotor.get("motor_front_right");
        motorLaunchLeft = hwMap.dcMotor.get("motor_launch_left");
        motorLaunchRight = hwMap.dcMotor.get("motor_launch_right");

        servoSetter = hwMap.servo.get("servo_setter");
        servoClutch = hwMap.servo.get("servo_clutch");
        servoCapRight = hwMap.servo.get("servo_cap_right");
        servoCapLeft = hwMap.servo.get("servo_cap_left");
        //servoColor = hwMap.servo.get("servo_pusher");


        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");


        // gyro = hwMap.gyroSensor.get("gyro");
        color = hwMap.colorSensor.get("color");
        color.enableLed(false);
        colorSensor = (ModernRoboticsI2cColorSensor) hwMap.get("color");
        //     colorSensorState = new FtcI2cDeviceState(colorSensor);

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorLaunchLeft.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorLaunchRight.setPower(0);
        motorLaunchLeft.setPower(0);
        motorIntake.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLaunchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLaunchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define and initialize ALL installed servos.

        servoSetter.setPosition(servoSetterDownPos);
        servoCapLeft.setPosition(servoCapLeftInitPos);
        servoCapRight.setPosition(servoCapRightInitPos);
        servoClutch.setPosition(servoClutchLockedPos);

        //    servoColor.setPosition(servoColorIn);


    }


    public void InitializeGyro()
    {
        Log.d("@@@InitGyro", "");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
        Log.d("@@@InitGyro", "set heading");
        gyro.calibrate();
        Log.d("@@@InitGyro", "calibrate");
        while (gyro.isCalibrating())
        {
            Log.d("@@@InitGyro", "in loop");
            opMode.sleep(50);
            opMode.idle();
        }
//        if (gyro != null)
//        {
//            gyrothread = new GyroThread(gyro);
//
//            gyrothread.calibrategyro();
//
//
//            gyrothread.start();
//        }

    }

//    public void DeregisterColorSensor()
//    {
//        colorSensor.getI2cController().deregisterForPortReadyCallback(colorSensor.getPort());
//    }
//
//    public void RegisterColorSensor()
//    {
//        colorSensor.getI2cController().registerForI2cPortReadyCallback(colorSensor, colorSensor.getPort());
//    }

    public void ResetCurrentHeading()
    {
        Log.d("@@@@enteredreset: ", "xxx");
        gyro.resetZAxisIntegrator();
        Log.d("@@@@@@resetZaxis: ", "xxx");

      /*  while (gyro.getIntegratedZValue() != 0)
        {
               Log.d("@@@@@@whileloop: ", "" + gyro.getIntegratedZValue());

            //     Log.d("@@@@@@try: ", "xxx");
            opMode.sleep(50);

        }*/

        targetHeading = 0;
    }

    // public void ResetCurrentHeading()
    // {

    //gyro.resetZAxisIntegrator();
    //  }
    public double GetCurrentHeading()
    {
        return -gyro.getIntegratedZValue();
    }

    // public double GetCurrentHeading()
    //   {
    //     return gyrothread.getCurrentHeading();
    // }

    public void setOpMode(RSLinearOpMode opMode)
    {
        this.opMode = opMode;
    }

    double calculateDrivePow(double totalDistance, double currentDistance, double maxPow)
    {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .2;
        double rampUpMinPow = .2;
        double rampDownCalcPow = 0;
        //distance in cm for speeding up
        double rampUpDistance = 20;
        //distance in cm for slowing down
        double rampDownDistance = 40;

        rampUpCalcPow = rampUpMinPow + (((maxPow - rampUpMinPow) / rampUpDistance) * currentDistance);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDistance) * (currentDistance - totalDistance));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);


        if (currentDistance > totalDistance / 2)
        {
            if (calculatedPow < minPow)
            {
                calculatedPow = minPow;

            }
        }
        else
        {
            if (calculatedPow < rampUpMinPow)
            {
                calculatedPow = rampUpMinPow;

            }
        }
        if (calculatedPow > maxPow)
        {
            calculatedPow = maxPow;
        }
        return Range.clip(calculatedPow, -1, 1);
    }

    double calculateSideDrivePow(double totalDistance, double currentDistance, double maxPow)
    {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .5;
        double rampUpMinPow = .5;
        double rampDownCalcPow = 0;
        //distance in cm for speeding up
        double rampUpDistance = 20;
        //distance in cm for slowing down
        double rampDownDistance = 10;

        rampUpCalcPow = rampUpMinPow + (((maxPow - rampUpMinPow) / rampUpDistance) * currentDistance);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDistance) * (currentDistance - totalDistance));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);


        if (currentDistance > totalDistance / 2)
        {
            if (calculatedPow < minPow)
            {
                calculatedPow = minPow;

            }
        }
        else
        {
            if (calculatedPow < rampUpMinPow)
            {
                calculatedPow = rampUpMinPow;

            }
        }
        if (calculatedPow > maxPow)
        {
            calculatedPow = maxPow;
        }
        return Range.clip(calculatedPow, -1, 1);
    }


    double LeftPowerCalc(double calculatedPow, double direction, double currentHeading)
    {
        double powerFactor = 30;
        double steerRatio = powerFactor / calculatedPow;

        double leftCalcPow = Range.clip((calculatedPow * direction) - (currentHeading / steerRatio), -1, 1);
        if ((calculatedPow * direction < 0 && leftCalcPow > 0) || (calculatedPow * direction > 0 && leftCalcPow < 0))
        {
            leftCalcPow = 0;
        }
        return (leftCalcPow);
    }

    double RightPowerCalc(double calculatedPow, double direction, double currentHeading)
    {
        double powerFactor = 30;
        double steerRatio = powerFactor / calculatedPow;

        double rightCalcPow = Range.clip((calculatedPow * direction) + (currentHeading / steerRatio), -1, 1);
        if ((calculatedPow * direction < 0 && rightCalcPow > 0) || (calculatedPow * direction > 0 && rightCalcPow < 0))
        {
            rightCalcPow = 0;
        }
        return (rightCalcPow);
    }

    private long Drive(double power, long distance, double direction, long initialheading) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        double kProportion = 0.04; //was 0.04
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;

        //      colorSensorState.setEnabled(false);

        //set current heading to zero
        ResetCurrentHeading();

        Log.d("@@@@@@@@@@@@@@@Drive: ", "" + distance * direction);
        Log.d("@@@Drive: Target head ", "" + targetHeading);

        //     Log.d("@@ResetCurrentHeading: ", "xxx");
        //use a while loop to keep motors going until desired heading reached

        //motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //      Log.d("@@@@@@@ResetBackRight: ", "xxx");
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //       Log.d("@@@@@@@@ResetBackLeft: ", "xxx");

        //This failed on the 2nd run
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //    Log.d("@Rnwithoutencod right: ", "xxx");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //    Log.d("@RUN_WITHOUT_ENCODERS: ", "xxx");


        encoderTarget = distance / onemotorclick;
        if (direction == left || direction == right)
        {
            encoderTarget = encoderTarget * 1.15;
        }

        while (abs(motorBackLeft.getCurrentPosition()) < encoderTarget &&
                abs(motorBackRight.getCurrentPosition()) < encoderTarget &&
                !IsRobotStalled(isLeftMotorStalled, isRightMotorStalled) && opMode.opModeIsActive() && !opMode.isStopRequested())
        {
            Log.d("@@@@@@@@BackLeft : ", "" + motorBackLeft.getCurrentPosition());
            Log.d("@@@@@@@@BackRight: ", "" + motorBackRight.getCurrentPosition());
            Log.d("@@@@@@@@Target   : ", "" + encoderTarget);


            // allow robot to drive on a heading other than zero
            currentHeading = GetCurrentHeading() + initialheading;
            Log.d("@@@@Heading ", "" + currentHeading);


            if (direction == forward || direction == backward)
            {

                calculatedPow = calculateDrivePow(distance, abs(motorBackLeft.getCurrentPosition()) * onemotorclick, power);

                //       Log.d("@@@Calc Pww ", "" + calculatedPow);

                leftCalculatedPow = LeftPowerCalc(calculatedPow, direction, currentHeading);

                rightCalculatedPow = RightPowerCalc(calculatedPow, direction, currentHeading);

                //       Log.d("@@@Left Calc Pow ", "" + leftCalculatedPow);
                //       Log.d("@@@Right Calc Pow ", "" + rightCalculatedPow);

                motorBackLeft.setPower(leftCalculatedPow);
                motorBackRight.setPower(rightCalculatedPow);
                motorFrontLeft.setPower(leftCalculatedPow);
                motorFrontRight.setPower(rightCalculatedPow);
            }
            else if (direction == left || direction == right)
            {
                double powerDifferential = (kProportion * currentHeading);
                calculatedPow = calculateSideDrivePow(distance, abs(motorBackLeft.getCurrentPosition()) * onemotorclick, power);

                //       Log.d("@@@Calc Pww ", "" + calculatedPow);

                //leftCalculatedPow = LeftPowerCalc(calculatedPow, direction / 2, 0);

                //rightCalculatedPow = RightPowerCalc(calculatedPow, direction / 2, 0);

                //reverse power for left/right power calculation
                calculatedPow = calculatedPow * direction / 2;

                motorBackLeft.setPower(-calculatedPow - powerDifferential);
                motorBackRight.setPower(calculatedPow + powerDifferential);
                motorFrontLeft.setPower(calculatedPow - powerDifferential);
                motorFrontRight.setPower(-calculatedPow + powerDifferential);
            }
            else
            {
                return 0;
            }

            //      opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
            //    opMode.telemetry.addData("Current Heading ", currentHeading);
            //   opMode.telemetry.addData("left power ", leftCalculatedPow);
            //  opMode.telemetry.addData("right power ", rightCalculatedPow);
            //   opMode.telemetry.update();


            leftMotorPos = motorBackLeft.getCurrentPosition();
            rightMotorPos = motorBackRight.getCurrentPosition();


            //if trying to move and not successfully moving
            if ((leftCalculatedPow != 0) && (abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 70 rounds through the loop (.? second)
                if (++leftStallCount == 70)
                {
                    Log.d("@STALLED LEFT: ", "xxx");
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if ((rightCalculatedPow != 0) && (abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 70 rounds through the loop
                if (++rightStallCount == 70)
                {
                    //right motor has stalled.
                    Log.d("@STALLED RIGHT: ", "xxx");
                    isRightMotorStalled = true;
                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            //opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        //opMode.waitForNextHardwareCycle();

        if (stallDetectionOn && (isLeftMotorStalled || isRightMotorStalled))
        {
            if (leftMotorPos > rightMotorPos)

                distance = (long) abs(leftMotorPos * onemotorclick);
            else
                distance = (long) abs(rightMotorPos * onemotorclick);
        }
        Log.d("@@@Distance ", "" + distance);
        //    colorSensorState.setEnabled(true);
        return (distance);
    }

    private long MultiDrive(double[] power, long[] distance, double[] direction, long[] initialHeading) throws InterruptedException
    {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        double previousLeftEncoder = 0;
        double previousRightEncoder = 0;
        double previousHeading = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos = 0;
        int rightMotorPos = 0;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightStallCutoff = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        long returnDistance = 0;
        double kProportion = 0.02; //was 0.06
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;

        //       colorSensorState.setEnabled(false);


        //     Log.d("@@ResetCurrentHeading: ", "xxx");
        //use a while loop to keep motors going until desired heading reached

        //motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //      Log.d("@@@@@@@ResetBackRight: ", "xxx");
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //       Log.d("@@@@@@@@ResetBackLeft: ", "xxx");

        //This failed on the 2nd run
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //    Log.d("@Rnwithoutencod right: ", "xxx");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //    Log.d("@RUN_WITHOUT_ENCODERS: ", "xxx");
        //
        // set current heading to zero
        ResetCurrentHeading();

        for (int step = 0; step < power.length; step++)
        {

            Log.d("@@@@@@@@@@@@@@@Drive: ", "" + distance[step]);
            Log.d("@@@Drive: Target head ", "" + targetHeading);


            initialHeading[step] = -initialHeading[step];

            encoderTarget = distance[step] / onemotorclick;
            if (direction[step] == left || direction[step] == right)
            {
                encoderTarget = encoderTarget * 1.15;//WAS 1.4!!! CHANGED TO TEST NEW WHEELS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            }

            //continue driving while encoders have not reached target position, the current heading has not reached target heading, and while the robot is not stalled
            while ((((abs(motorBackLeft.getCurrentPosition() - previousLeftEncoder) < encoderTarget) &&
                    (abs(motorBackRight.getCurrentPosition() - previousRightEncoder) < encoderTarget)) || abs(currentHeading) > 2) &&
                    !IsRobotStalled(isLeftMotorStalled, isRightMotorStalled) && opMode.opModeIsActive() && !opMode.isStopRequested())
            {
                Log.d("@@@@@@@@BackLeft : ", "" + motorBackLeft.getCurrentPosition());
                Log.d("@@@@@@@@BackRight: ", "" + motorBackRight.getCurrentPosition());
                Log.d("@@@@@@@@prvLeft : ", "" + previousLeftEncoder);
                Log.d("@@@@@@@@prvRight: ", "" + previousRightEncoder);
                Log.d("@@@@@@@@Target   : ", "" + encoderTarget);

                // allow robot to drive on a heading other than zero
                currentHeading = GetCurrentHeading() + initialHeading[step] - previousHeading;


                Log.d("@@@Heading ", "" + currentHeading);


                if (direction[step] == forward || direction[step] == backward)
                {

                    calculatedPow = calculateDrivePow(distance[step], abs(motorBackLeft.getCurrentPosition() - previousLeftEncoder) * onemotorclick, power[step]);

                    //       Log.d("@@@Calc Pww ", "" + calculatedPow);

                    leftCalculatedPow = LeftPowerCalc(calculatedPow, direction[step], currentHeading);

                    rightCalculatedPow = RightPowerCalc(calculatedPow, direction[step], currentHeading);

                    //       Log.d("@@@Left Calc Pow ", "" + leftCalculatedPow);
                    //       Log.d("@@@Right Calc Pow ", "" + rightCalculatedPow);

                    motorBackLeft.setPower(leftCalculatedPow);
                    motorBackRight.setPower(rightCalculatedPow);
                    motorFrontLeft.setPower(leftCalculatedPow);
                    motorFrontRight.setPower(rightCalculatedPow);
                }
                else if (direction[step] == left || direction[step] == right)
                {
                    double powerDifferential = (kProportion * currentHeading);
                    // calculatedPow = calculateSideDrivePow(distance[step], Math.abs(motorBackLeft.getCurrentPosition() - previousLeftEncoder) * onemotorclick, power[step]);
                    calculatedPow = power[step]; //changed to get rid of ramp up/down

                    //       Log.d("@@@Calc Pww ", "" + calculatedPow);

                    //leftCalculatedPow = LeftPowerCalc(calculatedPow, direction / 2, 0);

                    //rightCalculatedPow = RightPowerCalc(calculatedPow, direction / 2, 0);

                    // reverse power for left vs right
                    calculatedPow = calculatedPow * direction[step] / 2;

                    motorBackLeft.setPower(Range.clip(-calculatedPow - powerDifferential, -1, 1));
                    motorBackRight.setPower(Range.clip(calculatedPow + powerDifferential, -1, 1));
                    motorFrontLeft.setPower(Range.clip(calculatedPow - powerDifferential, -1, 1));
                    motorFrontRight.setPower(Range.clip(-calculatedPow + powerDifferential, -1, 1));


                    Log.d("@@@BL", "" + (motorBackLeft.getPower()));
                    Log.d("@@@BR", "" + (motorBackRight.getPower()));
                    Log.d("@@@FL", "" + (motorFrontLeft.getPower()));
                    Log.d("@@@FR", "" + (motorFrontRight.getPower()));
                }
                else
                {
                    return 0;
                }

                //      opMode.telemetry.addData("Current Encoder Position ", motorBackRight.getCurrentPosition());
                //    opMode.telemetry.addData("Current Heading ", currentHeading);
                //   opMode.telemetry.addData("left power ", leftCalculatedPow);
                //  opMode.telemetry.addData("right power ", rightCalculatedPow);
                //   opMode.telemetry.update();


                leftMotorPos = motorBackLeft.getCurrentPosition();
                rightMotorPos = motorBackRight.getCurrentPosition();


                //if trying to move and not successfully moving
                if ((leftCalculatedPow != 0) && (abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
                {
                    //if stalling for 50 rounds through the loop (.5 second)
                    if (++leftStallCount == 35)
                    {
                        Log.d("@STALLED LEFT: ", "xxx");
                        //left motor has stalled.
                        isLeftMotorStalled = true;

                    }
                }
                else
                {
                    // not stalled, reset stall counter
                    leftStallCount = 0;

                    isLeftMotorStalled = false;
                }
                //remembers encoder Pos for the next time through the loop
                leftStallPos = leftMotorPos;

                //if trying to move and not successfully moving
                if ((rightCalculatedPow != 0) && (abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
                {
                    //if stalling for 50 rounds through the loop
                    if (++rightStallCount == 35)
                    {
                        //right motor has stalled.
                        Log.d("@STALLED RIGHT: ", "xxx");
                        isRightMotorStalled = true;


                    }
                }
                else
                {
                    // not stalled, reset stall counter
                    rightStallCount = 0;

                    isRightMotorStalled = false;
                }
                //remembers encoder Pos for the next time through the loop
                rightStallPos = rightMotorPos;

                //opMode.waitForNextHardwareCycle();

            }

            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

            opMode.sleep(100); // WAS 1000

            previousLeftEncoder = motorBackLeft.getCurrentPosition();
            previousRightEncoder = motorBackRight.getCurrentPosition();
            previousHeading -= initialHeading[step]; //cumulative error
            Log.d("@@@previousHeading ", "" + previousHeading);

        }
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        //opMode.waitForNextHardwareCycle();

        if (stallDetectionOn && (isLeftMotorStalled || isRightMotorStalled))
        {
            if (leftMotorPos > rightMotorPos)

                returnDistance = (long) abs(leftMotorPos * onemotorclick);
            else
                returnDistance = (long) abs(rightMotorPos * onemotorclick);
        }
        Log.d("@@@Distance ", "" + distance);
        //   colorSensorState.setEnabled(true);
        return (returnDistance);
    }


    private boolean IsRobotStalled(boolean isLeftMotorStalled, boolean isRightMotorStalled)
    {
        if (stallDetectionOn)
        {
            return (isLeftMotorStalled || isRightMotorStalled);
        }
        else
        {
            return false;
        }

    }

    public long DriveForward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (Drive(power, distance, forward, 0));
    }

    public long DriveForwardToHeading(double power, long distance, long heading) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (Drive(power, distance, forward, -heading));
    }

    public long DriveBackward(double power, long distance) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (Drive(power, distance, backward, 0));
    }

    public long DriveBackwardToHeading(double power, long distance, long heading) throws InterruptedException
    {
        //Calling drive function and -1 is backward
        return (Drive(power, distance, backward, -heading));
    }

    public long DriveLeft(double power, long distance) throws InterruptedException
    {
        //Calling drive function and 1 is left

        return (Drive(power, distance, left, 0));
    }

    public long DriveRight(double power, long distance) throws InterruptedException
    {


        //Calling drive function and -1 is right
        return (Drive(power, distance, right, 0));
    }

    public long MultiDriveForwardToHeading(double[] power, long[] distance, double[] direction, long[] heading) throws InterruptedException
    {
        //Calling drive function and 1 is forward
        return (MultiDrive(power, distance, direction, heading));
    }

    public void DriveRightNoRamp(double power) throws InterruptedException
    {
        //  if (motorBackLeft.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0)
        {
            motorBackLeft.setPower(power);
            motorBackRight.setPower(-power);
            rightPow = -power;

            motorFrontLeft.setPower(-power);
            motorFrontRight.setPower(power);
            leftPow = -power;
        }

    }


    double rightPow;
    double leftPow;

    public void DriveLeftNoRamp(double power) throws InterruptedException
    {
        //     if (motorBackLeft.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0)
        {
            motorBackLeft.setPower(-power);
            motorBackRight.setPower(power);
            rightPow = power;

            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(-power);
            leftPow = power;
        }

    }

    public void StopDrive() throws InterruptedException
    {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

//    public double GetDeltaHeading()
//    {
//        double currentHeading = gyro.getHeading();
//        double deltaHeading = targetHeading - currentHeading;
//        if (deltaHeading > 180)
//        {
//            //MR gyro returns 0 through 359.  We need it to return -179 through 180
//            deltaHeading = deltaHeading - 360;
//        }
//        else if (deltaHeading < -180)
//        {
//            deltaHeading = deltaHeading + 360;
//        }
//        return deltaHeading;
//    }
//
//    public void ChangeTargetHeading(double degrees)
//    {
//        targetHeading = targetHeading + degrees;
//        if (targetHeading > 359)
//        {
//            targetHeading = targetHeading - 360;
//        }
//        else if (targetHeading < 0)
//        {
//            targetHeading = targetHeading + 360;
//        }
//    }

    double calculateTurnPow(double totalTurn, double currentHeading, double maxPow)
    {
        // if( currentHeading > 180)
        //     currentHeading = currentHeading - 360;

        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minSpinRampUpPow = .2;
        double minSpinRampDownPow = 0.1; //changed from 0.1 on 1/10
        double rampDownCalcPow = 0;
        //number of degrees for speeding up
        double rampUpDegrees = 30;
        //number of degrees for slowing down
        double rampDownDegrees = 60;

        rampUpCalcPow = minSpinRampUpPow + (((maxPow - minSpinRampUpPow) / rampUpDegrees) * abs(currentHeading));
        rampDownCalcPow = minSpinRampDownPow + (((minSpinRampDownPow - maxPow) / rampDownDegrees) * (abs(currentHeading) - totalTurn));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        //       Log.d("@@@@@@@ramp up calcPow:", "" + rampUpCalcPow);
        //       Log.d("@@@@@ramp down calcPow:", "" + rampDownCalcPow);

        if (calculatedPow < minSpinRampDownPow)
        {
            calculatedPow = minSpinRampDownPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    private long Spin(double power, long degrees, double direction) throws InterruptedException
    {

        Log.d("@@@@@@@@@@@@@@@Spin:", "" + degrees * direction);

        //   colorSensorState.setEnabled(false);
        ResetCurrentHeading();
        Log.d("@@@@@@@@@@@@@@@Spin:", "" + degrees * direction);
        // ChangeTargetHeading(degrees * direction);
        // Log.d("@@@@@@@@@@@Spin: Target", "" + targetHeading);
        double calculatedPow = 0;
        long turnAccuracy = 6;
        if (degrees <= 10)
        {
            turnAccuracy = 1;
        }
        //use a while loop to keep motors going until desired heading reached
        //    while (Math.abs(GetDeltaHeading()) > 8)
        while (abs(GetCurrentHeading()) < (degrees - turnAccuracy) && opMode.opModeIsActive() && !opMode.isStopRequested()) //3 changed from 8 on 1/10
        {
            calculatedPow = (calculateTurnPow(degrees, GetCurrentHeading(), power)) * direction;
            //calculatedPow = power * direction;
            motorFrontRight.setPower(-calculatedPow);
            motorBackRight.setPower(-calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

//            opMode.telemetry.addData("curr heading ", GetCurrentHeading());
//            opMode.telemetry.addData("target heading ", targetHeading);
//            opMode.telemetry.addData("delta heading ", GetDeltaHeading());
//            opMode.telemetry.addData("pow ", calculatedPow);
//            opMode.telemetry.update();


            Log.d("@@@@@@@@@@@@@@@Heading:", "" + GetCurrentHeading());
            //          Log.d("@@@@@@@@@@@@@@calcPow:", "" + calculatedPow);
            //    Log.d("@@@@@@@@@@@@@@@Delta:", "" + GetDeltaHeading());

            //          opMode.sleep(5);

        }


        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);


        opMode.sleep(200);

        Log.d("@@@@@@@@@Final Heading:", "" + GetCurrentHeading());
        //    Log.d("@@@@@@@@@Final Delta:", "" + GetDeltaHeading());

        //    colorSensorState.setEnabled(true);
        return (long) abs(GetCurrentHeading());
    }

    public long SpinRight(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction 1 is right
        //   if (myAlliance == Alliance.BLUE)
        return (Spin(power, degrees, 1));
        //    else
        //       return (Spin(power, degrees, -1));
    }

    public long SpinLeft(double power, long degrees) throws InterruptedException
    {
        //Calling spin function and direction -1 is left
        //   if (myAlliance == Alliance.BLUE)
        return (Spin(power, degrees, -1));
        //   else
        //       return (Spin(power, degrees, 1));
    }

    public void InitVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQPPMbD/////AAAAGQga5X8Nz0hpqD8HwZ2LFQoDU52KPaW9epBEBd4tzkkpfmN9OhLyS9ygj6LYSg3vgvWetSeZdbXkJj9vh2jXOZvTRVe6J7ptQIFIDrse5c6oPdZgt7D5wWacR/RKyj3qEIrO/1lKpF2N8E13psGc2tldpzLWIFwTyYR6WobgewXMpru5Y9rkIBd+ycr0WNkauaUt8uke7x3dznHqO3fgskOM+LqPrxg7vPzobQjO7wh2nH2hmGhjKZKeWjSrvz26XnK/mJRJar36SCDlEsx9h8qbmkjj7aN+S16knPkjLeTIzr0oNlI/KaS6j/3VC4QTzf0Rv9lgIvD2/3ZNB1VH0t01fR7FnoEEKL07wTTFc5fg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true); //enables RGB565 format for the image
        this.vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        fieldGraphics = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = fieldGraphics.get(0);
        wheels.setName("wheels");

        VuforiaTrackable tools = fieldGraphics.get(1);
        tools.setName("tools");

        VuforiaTrackable legos = fieldGraphics.get(2);
        legos.setName("legos");

        VuforiaTrackable gears = fieldGraphics.get(3);
        gears.setName("gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(fieldGraphics);

        float mmPerInch = 25.4f;
        float mmBotWidth = 17.5f * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float TARGET_HEIGHT = 160.0f;

        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix

                .translation(12.0f * mmPerInch, mmFTCFieldWidth / 2, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        //RobotLog.ii(TAG, "wheels=%s", format(wheelsLocationOnField));


        OpenGLMatrix legoLocationOnField = OpenGLMatrix

                .translation(-30.0f * mmPerInch, mmFTCFieldWidth / 2, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legoLocationOnField);
        //RobotLog.ii(TAG, "lego=%s", format(legoLocationOnField));


        OpenGLMatrix toolsLocationOnField = OpenGLMatrix

             //   .translation(-mmFTCFieldWidth/2, 30.0f*mmPerInch, TARGET_HEIGHT)
                .translation(30.0f * mmPerInch, mmFTCFieldWidth / 2, TARGET_HEIGHT)

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        //changed to 0 on 2/8 to simplify red math
                        //AngleUnit.DEGREES, 90, 90, 0));
                        AngleUnit.DEGREES, 90, 0, 0));
        tools.setLocation(toolsLocationOnField);
        //RobotLog.ii(TAG, "tools=%s", format(wheelsLocationOnField));




        OpenGLMatrix gearsLocationOnField = OpenGLMatrix

            //    .translation( -mmFTCFieldWidth/2,-12.0f*mmPerInch, TARGET_HEIGHT)
                .translation(-12.0f * mmPerInch, mmFTCFieldWidth / 2, TARGET_HEIGHT)

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        //changed to 0 on 2/8 to simplify red math
                        //AngleUnit.DEGREES, 90, 90, 0));
                        AngleUnit.DEGREES, 90, 0, 0));
        gears.setLocation(gearsLocationOnField);
        //RobotLog.ii(TAG, "gears=%s", format(legoLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                //The translation numbers are the position of the camera on the robot. It goes x, y, z.
                //The camera is 0 mm left of center, 225mm toward the robots front, and 0 mm below center.
                .translation(0, 225.0f, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 90, 0, 0));
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


        ((VuforiaTrackableDefaultListener) wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        ((VuforiaTrackableDefaultListener) tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


        /** Start tracking the data sets we care about. */
        fieldGraphics.activate();


        // HashMap<String, double[]> data = vuforia.
        float robotX = 0;
        float robotY = 0;
        float robotBearing = 0;
    }

    public void DeactivateVuforia()
    {
        fieldGraphics.deactivate();
    }

    public void ActivateVuforia()
    {
        fieldGraphics.activate();
    }

    public RobotPos GetStationaryVuforiaLocation()
    {
        RobotPos robotPos = new RobotPos();


        boolean foundAPicture = false;

        for (VuforiaTrackable trackable : allTrackables)
        {

            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
            //telemetry.addData("Gears raw x", vuforia.)
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
            {
                Log.d("@@@@VUGSVL", trackable.getName() + " IS VISIBLE");
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform == null)
                {
                    robotLocationTransform = lastLocation;
                }

                if (robotLocationTransform != null)
                {
                    VectorF trans = robotLocationTransform.getTranslation();
                    Orientation rot = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

// Robot position is defined by the standard Matrix translation (x and y)
                    robotPos.robotX = trans.get(0);
                    robotPos.robotY = trans.get(1);

// Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                    robotPos.robotBearing = rot.thirdAngle;
                    Log.d("@@@@VUGSVL", trackable.getName() + " IS GOOD");
                    lastRobotPos = robotPos;
                    lastLocation = robotLocationTransform;
                    foundAPicture = true;
                }
                //          else
                //                Log.d("@@@@VUGSVL", trackable.getName() + " IS NOT GOOD");
                break;
            }
            //    else
            //      Log.d("@@@@VUGSVL", trackable.getName() + " IS NOT VISIBLE");

        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (foundAPicture)
        {
            Log.d("@@@@VUGSVL", "LastRobotPos is GOOD");
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            //telemetry.addData("Pos", format(lastLocation));
            robotPos.goodPos = true;
        }
        else
        {
            Log.d("@@@@VUGSVL", "LastRobotPos is NULL");


            robotPos = lastRobotPos;
            robotPos.goodPos = false;

            //telemetry.addData("Pos", "Unknown");
        }

        return (robotPos);
    }

    Bitmap getVuforiaImage()
    {
        Image rgb = null;

        try
        {
            BlockingQueue<VuforiaLocalizer.CloseableFrame> queue = this.vuforia.getFrameQueue();
            VuforiaLocalizer.CloseableFrame frame = queue.take();

            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++)
            {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888)
                {
                    rgb = frame.getImage(i);

                    if (rgb != null)
                    {
                        //   telemetry.addData("In if loop", "The image is not null");
                        //   telemetry.update();

                        bitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.ARGB_8888);

                        ByteBuffer buf = rgb.getPixels();

                        byte[] imgRGB888 = new byte[buf.remaining()];

                        buf.get(imgRGB888);

                        int[] colors = new int[rgb.getWidth() * rgb.getHeight()];
                        int r, g, b;
                        for (int ci = 0; ci < colors.length; ci++)
                        {
                            r = (int) (0xFF & imgRGB888[3 * ci]);
                            g = (int) (0xFF & imgRGB888[3 * ci + 1]);
                            b = (int) (0xFF & imgRGB888[3 * ci + 2]);
                            colors[ci] = Color.rgb(r, g, b);
                        }

                        bitmap.setPixels(colors, 0, rgb.getWidth(), 0, 0, rgb.getWidth(), rgb.getHeight());


                    }


                    break;
                }//if
            }//for
            frame.close();
        } catch (InterruptedException e)
        {
        }

        return bitmap;
    }

    public RobotPos GetVuforiaLocation() throws InterruptedException
    {
        //force vuforia to find the image again
        lastLocation = null;

        RobotPos robotPos = null;
        int counter = 0;

        do
        {
            robotPos = GetStationaryVuforiaLocation();
//            telemetry.addData("Visible", robotPos.goodPos);
//            telemetry.addData("RobotX ", robotPos.robotX);
//            telemetry.addData("RobotY ", robotPos.robotY);
//            telemetry.addData("Heading ", robotPos.robotBearing);
//            telemetry.update();
//            idle();
            opMode.sleep(50);
            counter++;
        } while (robotPos.goodPos == false && counter <= 7);

        if (robotPos.goodPos == false)
        {
            SpinRight(1, 5);
            counter = 0;

            do
            {
                robotPos = GetStationaryVuforiaLocation();
//            telemetry.addData("Visible", robotPos.goodPos);
//            telemetry.addData("RobotX ", robotPos.robotX);
//            telemetry.addData("RobotY ", robotPos.robotY);
//            telemetry.addData("Heading ", robotPos.robotBearing);
//            telemetry.update();
//            idle();
                opMode.sleep(50);
                counter++;
            } while (robotPos.goodPos == false && counter <= 3);

        }
        if (robotPos.goodPos == false)
        {
            SpinLeft(1, 10);
            counter = 0;

            do
            {
                robotPos = GetStationaryVuforiaLocation();
//            telemetry.addData("Visible", robotPos.goodPos);
//            telemetry.addData("RobotX ", robotPos.robotX);
//            telemetry.addData("RobotY ", robotPos.robotY);
//            telemetry.addData("Heading ", robotPos.robotBearing);
//            telemetry.update();
//            idle();
                opMode.sleep(50);
                counter++;
            } while (robotPos.goodPos == false && counter <= 3);

        }
        int avgx = 0;
        int avgy = 0;
        int avgHeading = 0;
        counter = 0;
        if (robotPos.goodPos)
        {
            for (int i = 0; i < 10; i++)
            {

                robotPos = GetStationaryVuforiaLocation();
//            telemetry.addData("Visible", robotPos.goodPos);
//            telemetry.addData("RobotX ", robotPos.robotX);
//            telemetry.addData("RobotY ", robotPos.robotY);
//            telemetry.addData("Heading ", robotPos.robotBearing);
//            telemetry.update();
//            idle();
                opMode.sleep(30);
                if (robotPos.goodPos == true)
                {
                    counter++;
                    avgx += robotPos.robotX;
                    avgy += robotPos.robotY;
                    avgHeading += robotPos.robotBearing;
                }
            }
            if (counter > 0)
            {
                robotPos.robotX = avgx / counter;
                robotPos.robotY = avgy / counter;
                robotPos.robotBearing = avgHeading / counter;
            }
            else
            {
                robotPos.goodPos = false;
            }
        }
        else
            SpinRight(1, 5);


        return robotPos;
    }


    int getColorSensorRedValue()
    {
        return color.red();

    }

    int getColorSensorBlueValue()
    {
        return color.blue();

    }

    public void LaunchBall(int ballnum, double power)
    {
        int ballsLaunched = 0;


        motorLaunchLeft.setPower(power);
        motorLaunchRight.setPower(power);
        opMode.sleep(600);
        while (ballsLaunched < ballnum)
        {

            servoSetter.setPosition(servoSetterUpPos);
            opMode.sleep(750);
            servoSetter.setPosition(servoSetterDownPos);
            opMode.sleep(1000);
            ballsLaunched++;


        }
        motorLaunchLeft.setPower(0);
        motorLaunchRight.setPower(0);
    }

    public void scanLeftPressBeacon(Double power, String Alliance) throws InterruptedException
    {
        if (Alliance.equals("RED"))
        {
            int redCounter = 0;

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (redCounter < 3 && opMode.opModeIsActive() && !opMode.isStopRequested())
            {
                DriveLeftNoRamp(power);
                if (getColorSensorRedValue() >= 2)
                {
                    redCounter++;
                    opMode.sleep(20);
                }
                else
                    redCounter = 0;
            }
            StopDrive();

            opMode.sleep(100);

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int beaconFrontEdge = motorBackRight.getCurrentPosition();

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (getColorSensorRedValue() > 2 && opMode.opModeIsActive() && !opMode.isStopRequested())
            {
                DriveLeftNoRamp(.5);
                Log.d("Red value ", Integer.toString(getColorSensorRedValue()));
            }
            StopDrive();

            opMode.sleep(100);

            int beaconBackEdge = motorBackRight.getCurrentPosition();

            //    servoColor.setPosition(servoColorIn);

            DriveLeft(.5, 10);

            DriveBackward(.5, 7);

            DriveForward(.5, 7);
        }
    }


    public void scanRightPressBeacon(double power, String Alliance) throws InterruptedException
    {

        beaconFound = false;

        if (Alliance.equals("BLUE"))
        {
            int blueCounter = 0;
            int redCounter = 0;
            //double previousencodertarget = motorBackRight.getCurrentPosition();


            long startTime = Calendar.getInstance().getTimeInMillis();
            long currentTime = Calendar.getInstance().getTimeInMillis();


            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            DriveRightNoRamp(power);


            while (blueCounter < 5 && opMode.opModeIsActive() && !opMode.isStopRequested() && currentTime - startTime < 2000)
            {

                if (getColorSensorBlueValue() >= 4 && getColorSensorRedValue() < 1)
                {
                    blueCounter++;
                    opMode.sleep(20);
                }
                else
                    blueCounter = 0;

                if (getColorSensorRedValue() > 4)
                {
                    redCounter++;
                    opMode.sleep(20);
                    if (redCounter > 5)
                    {
                        wrongColorFirst = true;
                    }
                }
                //   Log.d("@@@@@@@@@Color Blue ", "" + getColorSensorBlueValue());
                //     Log.d("@@@@@@@@@blueCounter ", "" + blueCounter);

                currentTime = Calendar.getInstance().getTimeInMillis();
            }
            StopDrive();
            opMode.sleep(100);

            if(blueCounter == 5){
                beaconFound = true;

                //push the button
                stallDetectionOn = true;
                DriveBackward(.8, 20);
                stallDetectionOn = false;
                DriveForward(.7, 9);
                opMode.sleep(50);
            }
            if(beaconFound == false){
                DriveRight(power, (long) abs(motorBackLeft.getCurrentPosition() * onemotorclick));
            }




            //int beaconFrontEdge = motorBackRight.getCurrentPosition();

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            opMode.sleep(100);

            //int beaconBackEdge = motorBackRight.getCurrentPosition();
            //DriveRight(.4,5);
            //    servoColor.setPosition(servoColorPush);

            //   servoColor.setPosition(servoColorIn);
        }


        else
        {
            int redCounter = 0;
            int blueCounter = 0;

            long startTime = Calendar.getInstance().getTimeInMillis();
            long currentTime = Calendar.getInstance().getTimeInMillis();

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (redCounter < 5 && opMode.opModeIsActive() && !opMode.isStopRequested() && currentTime - startTime < 2000)
            {
                DriveRightNoRamp(power);
                if (getColorSensorRedValue() >= 2 && getColorSensorBlueValue() < 1)
                {
                    redCounter++;
                    opMode.sleep(20);
                }
                else
                    redCounter = 0;
                Log.d("@@@@@@@@@Color Red ", "" + getColorSensorRedValue());
                Log.d("@@@@@@@@@redCounter ", "" + redCounter);

                if (getColorSensorBlueValue() > 4)
                {
                    blueCounter++;
                    opMode.sleep(20);
                    if (blueCounter > 5)
                    {
                        wrongColorFirst = true;
                    }
                }

                currentTime = Calendar.getInstance().getTimeInMillis();
            }
            StopDrive();
            opMode.sleep(100);
            if(redCounter == 5){
                beaconFound = true;

                //push button
                stallDetectionOn = true;
                DriveBackward(.7, 20);
                stallDetectionOn = false;
                DriveForward(.7, 9);
            }

            if(beaconFound == false){
                DriveRight(power, (long) abs(motorBackLeft.getCurrentPosition() * onemotorclick));
            }


            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //int beaconFrontEdge = motorBackRight.getCurrentPosition();

            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            opMode.sleep(100);

           // int beaconBackEdge = motorBackRight.getCurrentPosition();
            //DriveRight(.4,5);
            //     servoColor.setPosition(servoColorPush);

            //     servoColor.setPosition(servoColorIn);
        }
    }


}


