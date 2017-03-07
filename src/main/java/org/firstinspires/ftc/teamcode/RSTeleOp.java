package org.firstinspires.ftc.teamcode;/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Calendar;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 * <p>
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Tele-op", group = "RS_Tele-op")
public class RSTeleOp extends LinearOpMode
{

    /* Declare OpMode members. */
    RSRobot robot = new RSRobot();              // Use a K9'shardware

    @Override
    public void runOpMode() throws InterruptedException
    {
        double backLeftPow;
        double backRightPow;
        double frontLeftPow;
        double frontRightPow;

        double y1;
        double y2;
        double x1;
        double x2;

        boolean gmpd2aPressed = false;
        boolean gmpd2DpadUpPressed = false;
        boolean gmpd2DpadDownPressed = false;
        boolean gmpd1aPressed = false;
        boolean gmpd1bPressed = false;


        double defaultRightSpinnerSpeed = .5;
        double defaultLeftSpinnerSpeed = .5;
        double rightSpinnerSpeed = defaultRightSpinnerSpeed;
        double leftSpinnerSpeed = defaultLeftSpinnerSpeed;
        double negativeShooterSpeed = -0.4;
        boolean shooterRunning = false;
        boolean negativeshooter = false;

        //   double servoColorPos = .5;

        boolean continuousIntakeIn = false;
        boolean continuousIntakeOut = false;
        boolean clutchEngaged = false;
        double intakeSpeed = .7;
        double liftSpeed = 1;
        double lowerSpeed = -.3;


        boolean drivingForward = true;
        boolean drivingLeft = false;
        boolean fineControl = false;
        double fineControlMultiplier = 1;
        double x2FineControlMultiplier = 1;

        double leftPreviousEncoder = 0;
        double leftNewEncoder = 0;
        double rightPreviousEncoder = 0;
        double rightNewEncoder = 0;
        long leftNewTime = 0;
        long leftPreviousTime = 0;
        long rightNewTime = 0;
        long rightPreviousTime = 0;
        double leftDelta = 0;
        double rightDelta = 0;
        int encoderArraySize = 5;
        EncoderData avgrightEncoder[] = new EncoderData[encoderArraySize];
        EncoderData avgleftEncoder[] = new EncoderData[encoderArraySize];
        int leftEncoderIndex =0;
        int rightEncoderIndex=0;
        boolean rightEncoderArrayFull=false;
        boolean leftEncoderArrayFull = false;
        long rightTime=0;
        long leftTime=0;
        double rightAROC=0;
        double leftAROC=0;
        double servoSetterPos = robot.servoSetterDownPos;

        /* InitializeGyro the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

    for( int i=0;i<encoderArraySize;i++)
    {
        avgrightEncoder[i] = new EncoderData();
        avgleftEncoder[i]= new EncoderData();
    }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            if (drivingLeft)
            {
                x1 = gamepad1.left_stick_y;
                y1 = gamepad1.left_stick_x; //flip x and y axes to drive left
            }
            else
            {
                x1 = gamepad1.left_stick_x;
                y1 = -gamepad1.left_stick_y; //y is inverse
            }

            y2 = -gamepad1.right_stick_y; //y is inverse
            x2 = gamepad1.right_stick_x;

            if (x1 >= -0.1 && x1 <= 0.1)
            {
                x1 = 0;
            }
            if (x2 >= -0.1 && x2 <= 0.1)
            {
                x2 = 0;
            }
            if (y1 >= -0.1 && y1 <= 0.1)
            {
                y1 = 0;
            }
            if (y2 >= -0.1 && y2 <= 0.1)
            {
                y2 = 0;
            }

            //switch directions
            if (gamepad1.dpad_up)
            {
                drivingForward = true;
                drivingLeft = false;
            }
            else if (gamepad1.dpad_down)
            {
                drivingForward = false;
                drivingLeft = false;
            }
            else if (gamepad1.dpad_left)
            {
                drivingLeft = true;
            }


            if (gamepad1.x)
            {
                fineControl = true;
                fineControlMultiplier = .45;
                x2FineControlMultiplier = .35;

            }
            else if (gamepad1.y)
            {
                fineControl = false;
                fineControlMultiplier = 1;
                x2FineControlMultiplier = 1;
            }

            x2 = x2 * x2FineControlMultiplier;

            frontRightPow = (y1 - x2 - x1) * fineControlMultiplier;
            backRightPow = (y1 - x2 + x1) * fineControlMultiplier;
            frontLeftPow = (y1 + x2 + x1) * fineControlMultiplier;
            backLeftPow = (y1 + x2 - x1) * fineControlMultiplier;


            if (drivingForward || drivingLeft)
            {
                robot.motorFrontLeft.setPower(frontLeftPow);
                robot.motorFrontRight.setPower(frontRightPow);
                robot.motorBackLeft.setPower(backLeftPow);
                robot.motorBackRight.setPower(backRightPow);
            }
            else
            {
                robot.motorFrontLeft.setPower(-backRightPow);
                robot.motorFrontRight.setPower(-backLeftPow);
                robot.motorBackLeft.setPower(-frontRightPow);
                robot.motorBackRight.setPower(-frontLeftPow);
            }


            //a is backleft
            //b is back right
            //x is front left
            //y is front right


            //turn spinners on and off
            if (gamepad2.a)
            {
                if (!gmpd2aPressed)
                {
                    gmpd2aPressed = true;
                    if (!shooterRunning)
                    {
                        shooterRunning = true;
                    }
                    else
                    {
                        shooterRunning = false;
                    }
                }
            }
            else
            {
                gmpd2aPressed = false;
            }

            //speed spinners up
            if (gamepad2.dpad_up)
            {
                if (!gmpd2DpadUpPressed)
                {
                    gmpd2DpadUpPressed = true;
                    if (rightSpinnerSpeed < 1)
                    {
                        rightSpinnerSpeed = rightSpinnerSpeed + 0.1;
                        leftSpinnerSpeed = leftSpinnerSpeed + 0.1;
                    }
                }
            }
            else
            {
                gmpd2DpadUpPressed = false;
            }

            //slow spinners down
            if (gamepad2.dpad_down)
            {
                if (!gmpd2DpadDownPressed)
                {
                    gmpd2DpadDownPressed = true;
                    if (rightSpinnerSpeed > .4)
                    {
                        rightSpinnerSpeed = rightSpinnerSpeed - 0.1;
                        leftSpinnerSpeed = leftSpinnerSpeed - 0.1;
                    }
                }
            }
            else
            {
                gmpd2DpadDownPressed = false;
            }


            //set spinner speed
            if (shooterRunning)
            {
                robot.motorLaunchLeft.setPower(leftSpinnerSpeed);
                robot.motorLaunchRight.setPower(rightSpinnerSpeed);
            }
            else
            {
                if (negativeshooter)
                {
                    robot.motorLaunchLeft.setPower(negativeShooterSpeed);
                    robot.motorLaunchRight.setPower(negativeShooterSpeed);
                }
                else
                {
                    robot.motorLaunchLeft.setPower(0);
                    robot.motorLaunchRight.setPower(0);
                }
            }

            //telemetry.addData("shooter speed", "%.2f", rightSpinnerSpeed);
            //telemetry.update();


            if (gamepad2.b)
            {
                negativeshooter = true;
                shooterRunning = false;
            }
            else
            {
                negativeshooter = false;
            }
            //fine tuner for color servo
            if (gamepad2.dpad_right)
            {
                if (servoSetterPos < .99)
                {
                    servoSetterPos = servoSetterPos + .01;
                    robot.servoSetter.setPosition(servoSetterPos);
                }
            }

            if (gamepad2.dpad_left)
            {
                if (servoSetterPos > 0.01)
                {
                    servoSetterPos = servoSetterPos - .01;
                    robot.servoSetter.setPosition(servoSetterPos);
                }
            }


/*
            if (gamepad2.x)
            {
                servoSetterPos = robot.servoSetterUpPos;
                robot.servoSetter.setPosition(servoSetterPos);
            }

            if (gamepad2.b)
            {
                servoSetterPos = robot.servoSetterDownPos;
                robot.servoSetter.setPosition(servoSetterPos);
            }

            telemetry.addData("servo setter", "%.2f", servoSetterPos);*/
            telemetry.addData("color sensor red", "%d", robot.color.red());
            telemetry.addData("color sensor blue", "%d", robot.color.blue());
     //       telemetry.update();

            //continuous intake in
            if (gamepad1.a)
            {
                if (!gmpd1aPressed)
                {
                    gmpd1aPressed = true;
                    if (continuousIntakeIn)
                    {
                        continuousIntakeIn = false;
                    }
                    else
                    {
                        continuousIntakeIn = true;
                    }
                }
            }
            else
            {
                gmpd1aPressed = false;
            }

            //continous intake out
            if (gamepad1.b)
            {
                if (!gmpd1bPressed)
                {
                    gmpd1bPressed = true;
                    if (continuousIntakeOut)
                    {
                        continuousIntakeOut = false;
                    }
                    else
                    {
                        continuousIntakeOut = true;
                    }
                }
            }
            else
            {
                gmpd1bPressed = false;
            }


            //set the intake power
            if (gamepad1.right_trigger > .1)
            {
                continuousIntakeIn = false;
                continuousIntakeOut = false;
                if (clutchEngaged)
                {
                    robot.motorIntake.setPower(liftSpeed);
                    robot.motorIntake2.setPower(liftSpeed);
                }
                else
                {
                    robot.motorIntake.setPower(intakeSpeed);
                    robot.motorIntake2.setPower(intakeSpeed);
                }
            }
            else if (gamepad1.left_trigger > .1)
            {
                continuousIntakeIn = false;
                continuousIntakeOut = false;
                if (clutchEngaged)
                {
                    robot.motorIntake.setPower(lowerSpeed);
                    robot.motorIntake2.setPower(lowerSpeed);
                }
                else
                {
                    robot.motorIntake.setPower(-intakeSpeed);
                    robot.motorIntake2.setPower(-intakeSpeed);
                }
            }
            else if (continuousIntakeIn)
            {
                if (clutchEngaged)
                {
                    robot.motorIntake.setPower(liftSpeed);
                    robot.motorIntake2.setPower(liftSpeed);
                }
                else
                {
                    robot.motorIntake.setPower(intakeSpeed);
                    robot.motorIntake2.setPower(intakeSpeed);
                }
                continuousIntakeOut = false;
            }
            else if (continuousIntakeOut)
            {
                if (clutchEngaged)
                {
                    robot.motorIntake.setPower(lowerSpeed);
                    robot.motorIntake2.setPower(lowerSpeed);
                }
                else
                {
                    robot.motorIntake.setPower(-intakeSpeed);
                    robot.motorIntake2.setPower(-intakeSpeed);
                }
            }
            else
            {
                robot.motorIntake.setPower(0);
                robot.motorIntake2.setPower(0);
            }
            if (gamepad1.left_bumper)
            {
                robot.servoCapLeft.setPosition(robot.servoCapLeftParallel);
                robot.servoCapRight.setPosition(robot.servoCapRightParallel);

            }
            if (gamepad1.right_bumper)
            {
                robot.servoCapLeft.setPosition(robot.servoCapLeftHoldPos);
                robot.servoCapRight.setPosition(robot.servoCapRightHoldPos);

            }
            //automatic shoot

            if (gamepad2.y)
            {
                robot.servoSetter.setPosition(robot.servoSetterUpPos);
                sleep(500);
                robot.servoSetter.setPosition(robot.servoSetterDownPos);
            }
            if (gamepad2.left_bumper)
            {
                robot.servoClutch.setPosition(robot.servoClutchEngagedPos);
                clutchEngaged = true;

            }
            if (gamepad2.right_bumper)
            {
                robot.servoClutch.setPosition(robot.servoClutchDisengagePos);
                clutchEngaged = false;

            }
            leftNewTime = Calendar.getInstance().getTimeInMillis();
            leftNewEncoder = robot.motorLaunchLeft.getCurrentPosition();
            rightNewTime = Calendar.getInstance().getTimeInMillis();
            rightNewEncoder = robot.motorLaunchRight.getCurrentPosition();


            if(rightNewEncoder != rightPreviousEncoder )
            {


                avgrightEncoder[rightEncoderIndex].delta =  (rightNewEncoder - rightPreviousEncoder);
                avgrightEncoder[rightEncoderIndex].millis = (rightNewTime - rightPreviousTime);

                if(++rightEncoderIndex==encoderArraySize){
                    rightEncoderIndex=0;
                    rightEncoderArrayFull=true;
                }
                rightPreviousTime = rightNewTime;
                if(rightEncoderArrayFull)
                {
                    rightTime = 0;
                    rightDelta = 0;
                    for (int i = 0; i < encoderArraySize; i++)
                    {
                        rightDelta += avgrightEncoder[i].delta;
                        rightTime += avgrightEncoder[i].millis;
                    }
                    rightDelta = rightDelta/encoderArraySize;
                    rightAROC = rightDelta/rightTime;
                }
            }


            if(leftNewEncoder != leftPreviousEncoder )
            {
                leftDelta = (leftNewEncoder - leftPreviousEncoder)/(leftNewTime - leftPreviousTime);
                leftPreviousTime = leftNewTime;
            }

            telemetry.addData("rightEnc ", rightNewEncoder);
            telemetry.addData("leftEnc ", leftNewEncoder);

            telemetry.addData("rightDelta ", rightDelta);
            telemetry.addData("rightAROC ", rightAROC);
            telemetry.addData("leftDelta ", leftDelta);
            telemetry.update();

            rightPreviousEncoder = rightNewEncoder;
            leftPreviousEncoder = leftNewEncoder;



            /*

            //motor_1
            if(gamepad1.a)
                robot.motorBackLeft.setPower(50);
            else
                robot.motorBackLeft.setPower(0);

            //motor_2
            if(gamepad1.b)
                robot.motorBackRight.setPower(50);
            else
                robot.motorBackRight.setPower(0);

            //motor_3
            if(gamepad1.x)
                robot.motorFrontLeft.setPower(50);
            else
                robot.motorFrontLeft.setPower(0);

            //motor_4
            if(gamepad1.y)
                robot.motorFrontRight.setPower(50);
            else
                robot.motorFrontRight.setPower(0);

            //motor_1

            if(gamepad1.dpad_down)
                robot.motorIntake.setPower(50);
            else
                robot.motorIntake.setPower(0);


            if (gamepad1.dpad_right)
                robot.motorLaunchRight.setPower(50);
            else
                robot.motorLaunchRight.setPower(0);

            if (gamepad1.dpad_left)
                robot.motorLaunchLeft.setPower(50);
            else
                robot.motorLaunchLeft.setPower(0);



*/



            /* C code of mecanum driving

           motor[frontRight] = y1_exp - x2_exp - x1_exp;
		motor[backRight] =  y1_exp - x2_exp + x1_exp;
		motor[frontLeft] = y1_exp + x2_exp + x1_exp;
		motor[backLeft] =  y1_exp + x2_exp - x1_exp;


             */



            /* PREMADE SERVO CODE -- CURRENTLY UNNEEDED

            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                armPosition += ARM_SPEED;
            else if (gamepad1.y)
                armPosition -= ARM_SPEED;

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

            */
        }
    }
}
