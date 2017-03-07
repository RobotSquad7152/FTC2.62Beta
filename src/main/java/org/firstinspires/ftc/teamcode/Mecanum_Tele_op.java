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
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@TeleOp(name="Mecanum Telo-op", group="K9bot")
public class Mecanum_Tele_op extends LinearOpMode
{

    /* Declare OpMode members. */
  //  RSRobot robot           = new RSRobot();              // Use a K9'shardware

    @Override
    public void runOpMode() throws InterruptedException {
        double backLeftPow;
        double backRightPow;
        double frontLeftPow;
        double frontRightPow;

        double y1;
        double y2;
        double x1;
        double x2;

        /* InitializeGyro the hardware variables.
         * The init() method of the hardware class does all the work here
         */
     //   robot.init(hardwareMap);
        DcMotor motor1 = hardwareMap.dcMotor.get("motor_1"); //
        DcMotor motor2 = hardwareMap.dcMotor.get("motor_2"); //
        DcMotor motor3 = hardwareMap.dcMotor.get("motor_3"); //
        DcMotor motor4 = hardwareMap.dcMotor.get("motor_4"); //


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.x)
                motor1.setPower(.50);
            else
                motor1.setPower(0);


            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
//            y1 = -gamepad1.left_stick_y; //y is inverse
//            y2 = -gamepad1.right_stick_y; //y is inverse
//            x1 = gamepad1.left_stick_x;
//            x2 = gamepad1.right_stick_x;
//
//            if(x1 >= -0.1&& x1 <= 0.1){
//                x1 =0;
//            }
//            if(x2 >= -0.1&& x2 <= 0.1){
//                x2 =0;
//            }
//            if(y1 >= -0.1&& y1 <= 0.1){
//                y1 =0;
//            }
//            if(y2 >= -0.1&& y2 <= 0.1){
//                y2 =0;
//            }
//
//            frontRightPow = y1 - x2 - x1;
//            backRightPow = y1 - x2 + x1;
//            frontLeftPow = y1 + x2 + x1;
//            backLeftPow = y1 + x2 - x1;
//


//            robot.motorFrontRight.setPower(frontRightPow);
//            robot.motorBackLeft.setPower(backLeftPow);
//            robot.motorBackRight.setPower(backRightPow);

/*

            //a is backleft
            //b is back right
            //x is front left
            //y is front right

            //motor_1
            if(gamepad1.a)
                robot.frontLeftMotor.setPower(50);
            else
                robot.frontLeftMotor.setPower(0);

            //motor_2
            if(gamepad1.b)
                robot.frontRightMotor.setPower(50);
            else
                robot.frontRightMotor.setPower(0);

            //motor_3
            if(gamepad1.x)
                robot.backLeftMotor.setPower(50);
            else
                robot.backLeftMotor.setPower(0);

            //motor_4
            if(gamepad1.y)
                robot.backRightMotor.setPower(50);
            else
                robot.backRightMotor.setPower(0);

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
