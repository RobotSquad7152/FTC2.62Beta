package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import android.util.Log;

/**
 * Created by Tony on 2/16/2016.
 */
public abstract class RSLinearOpMode extends LinearOpMode
{
    public RSRobot robot;

    /*public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorSlide;
    public DcMotor motorSpinner;
    public DcMotor motorTape;
    public DcMotor motorBucket;
    public Servo servoChurro;
    public Servo servoHopper;
    public Servo servoClimberRight;
    public Servo servoClimberLeft;
    public Servo servoDoor;
    public Servo servoLeftZip;
    public Servo servoRightZip;
    public Servo servoClutch;
    public Servo servoTape;


    public DcMotorController motorControllerFrontDrive;
    public DcMotorController motorControllerRearDrive;
    public ModernRoboticsI2cGyro gyro;
    public ColorSensor sensorRGB;*/
    public long zteTargetY=1660;
    public long motoTargetY= 1750;
    public long legosLeftTargetX=-800;
    public long legosRightTargetX=-600;
    public long gearsLeftTargetX =-350;
    public long gearsRightTargetX;
    public long wheelsLeftTargetX = 250;
    public long wheelsRightTargetX = 490;
    public long toolsLeftTargetX = 700;
    public long toolsRightTargetX;
    @Override
    public abstract void runOpMode() throws InterruptedException;

    public RSRobot InitHardware()
    {
    Log.d("@@@InitHardware","");

        robot = new RSRobot();
        Log.d("@@@InitHardware","New robot");

        robot.init(hardwareMap);
        Log.d("@@@InitHardware","init");
        robot.setOpMode(this);

        Log.d("@@@InitHardware","set opmode");
        // calibrate gyro etc.
        robot.InitializeGyro();

        Log.d("@@@InitHardware","init gyro");

        return robot;
    }

    public RSRobot InitNullHardware()
    {

        robot = new RSRobot();

        robot.initNull(hardwareMap);

        robot.setOpMode(this);

        // calibrate gyro etc.
         robot.InitializeGyro();


        return robot;
    }

}


