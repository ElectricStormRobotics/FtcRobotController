/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ServiceLoader;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Into daDeep", group="Linear OpMode")
//@Disabled
public class fieldcentricopmode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private ElapsedTime ledtimer = new ElapsedTime();
    private DcMotor Front_Right = null;  //  Used to control the left front drive wheel
    private DcMotor Front_Left = null;  //  Used to control the right front drive wheel
    private DcMotor Back_Left = null;  //  Used to control the left back drive wheel
    private DcMotor Back_Right = null;  //  Used to control the right back drive wheel
    private DcMotor Slide = null;  //  Used to control both of the slides
    private DcMotor PIVOT = null; // Lead Screw manipulator


    private Servo elbow = null; // Brings Arm attachment up and down
    private Servo wrist = null; // Brings Arm attachment up and down

    private Servo twist = null; // Twists Intake

    private CRServo right_CR; //Right Intake wheel
    private CRServo left_CR; //Left Intake wheel

    // private neopixel_i2c LED_strip;
   // private boolean led_race=true;

    //private boolean led_off = true;
    //private boolean end_game = false;
    //private int pix_on = 3; //sets all leds on (2 sets every other, 3 sets every 3rd)
    double boost = 0.55;
    double elbowdown = 0.6;
    double elbowup = 0.0;
    double zeroFlip = 0.32;
    double ninetyDe = 0.0;
    double one80 = .65;
    double two70 =.9;

    double maxExten = -6000;
    boolean armup =  false;

    double g = (0.000000000001); // Slide is all the way down
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Front_Right  = hardwareMap.get(DcMotor.class, "Front_Right");
        Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
        Back_Left  = hardwareMap.get(DcMotor.class, "Back_Left");
        Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
        Slide = hardwareMap.get(DcMotor.class, "slide");
        PIVOT = hardwareMap.get(DcMotor.class, "PIVOT");
        left_CR = hardwareMap.get(CRServo.class, "left_CR");
        right_CR = hardwareMap.get(CRServo.class, "right_CR");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        twist = hardwareMap.get(Servo.class, "twist");




        // LED_strip  = hardwareMap.get(neopixel_i2c.class, "led_strip");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
      //  Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // PIVOT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIVOT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Left.setDirection(DcMotor.Direction.REVERSE);
        Back_Left.setDirection(DcMotor.Direction.REVERSE);
        Front_Right.setDirection(DcMotor.Direction.FORWARD);
        Back_Right.setDirection(DcMotor.Direction.FORWARD);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        left_CR.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)


        int led_countdown = 0;
        int num_pixels = 300; // number of pixels (LEDs) in to work with.


        waitForStart();
        elbow.setPosition(0);
        wrist.setPosition(0);
        twist.setPosition(zeroFlip);
        runtime.reset();
        //ledtimer.reset();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
       // LED_strip.doInitialize();
        imu.initialize(parameters);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double PivotAngle = PIVOT.getCurrentPosition() / 80; // Ticks per degree


            double threshold = 0;
                threshold = -DesiredSlideTicks(PIVOT.getCurrentPosition());

                int red = 0xFF;
                int blue = 0x00;
                int green = 0x00;
/*
                // Doesn't need to be the whole strip, but needs to be all the LEDs planned to be used
                if((runtime.seconds()%60)>= 59)// reset to off
                {
                    LED_strip.writeLEDstrip(num_pixels, 0x00, 0x00, 0x00,pix_on);
                    led_countdown = 0;
                    telemetry.addData("","60 seconds");
                    telemetry.update();
                }

                else if((runtime.seconds()%60)> 50 && led_countdown<5)// 50 secs teal
                {
                    LED_strip.writeLEDstrip(num_pixels, 0x00, 0xFF, 0xff,pix_on);
                    led_countdown = 5;
                    telemetry.addData("","50 seconds");
                    telemetry.update();
                }

                else if((runtime.seconds()%60)> 40 && led_countdown<4)// 40 secs yellow
                {
                    LED_strip.writeLEDstrip(num_pixels, 0xFF, 0xFF, 0x00,pix_on);
                    led_countdown = 4;
                    telemetry.addData("","40 seconds");
                    telemetry.update();
                }

                else if((runtime.seconds()%60)> 30 && led_countdown<3)// 30 seconds red
                {
                    LED_strip.writeLEDstrip(num_pixels, 0xFF, 0x00, 0x00,pix_on);
                    led_countdown = 3;
                    telemetry.addData("","30 seconds");
                    telemetry.update();
                }
                else if ((runtime.seconds()%60)> 20 && led_countdown<2)//20 seconds blue
                // sets lights to blue
                {
                    LED_strip.writeLEDstrip(num_pixels, 0x00, 0x00, 0xff,pix_on);
                    led_countdown = 2;
                    telemetry.addData("","25 seconds");
                    telemetry.update();
                }
                else if ((runtime.seconds()%60)> 10 && led_countdown<1)//10 secs green
                // sets lights to green
                {
                    LED_strip.writeLEDstrip(num_pixels, 0x00, 0xff, 0x00,pix_on);
                    led_countdown = 1;
                    telemetry.addData("","15 seconds");
                    telemetry.update();
                }
                else {}


                // telemetry.addData("Status", "LED Timer: " + ledtimer.toString());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }


 */

            //TODO Code All da Arm Stuff

            // Intake
            if (gamepad1.a) {
                left_CR.setPower(1);
                right_CR.setPower(1);

            } else if (gamepad1.b) {
                left_CR.setPower(-1);
                right_CR.setPower(-1);

            } else {
                left_CR.setPower(0);
                right_CR.setPower(0);
            }
            // Driver 2 controls

            // Elbow
          /*  if (gamepad2.dpad_down && PivotAngle > 45 ) {
                elbow.setPosition(elbowdown);
                wrist.setPosition(elbowdown);
            }

            if (gamepad2.dpad_up && PivotAngle > 45) {
                elbow.setPosition(elbowup);
                wrist.setPosition(elbowup);
            }

           */
            if (gamepad2.dpad_up) {
                elbow.setPosition(elbowup);
                wrist.setPosition(elbowup);
            }

            //Twist/wrist
            if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                twist.setPosition(ninetyDe);
            }
            else if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                twist.setPosition(one80);
            }
            else if (gamepad2.right_bumper && gamepad2.left_bumper) {
                twist.setPosition(two70);
            }
            else {
                twist.setPosition(zeroFlip);
            }

            //Arm Code
            if(PIVOT.getCurrentPosition() < -5000) {
                armup = true;
            }
            else{
                    armup =false;
            }

            //DONE WITH DRIVER 1
            if (!armup) {
                boost = .65;
                maxExten = -6000;
                /*if (Slide.getCurrentPosition() < maxExten) {
                    Slide.setPower(gamepad2.right_stick_y);
                }
                else if (Slide.getCurrentPosition() > maxExten && gamepad2.right_stick_y >.1) {
                    Slide.setPower(0);
                } else if (Slide.getCurrentPosition() > maxExten && gamepad2.right_stick_y < .1) {
                    Slide.setPower(gamepad2.right_stick_y);
                }*/
                elbowup = 0.0;
            }
            if (armup) {
                //Slide.setPower(gamepad2.right_stick_y + (g * Slide.getCurrentPosition()));
                boost = .25;
                elbowup = 0.0;
                maxExten = -8000;
            }
            //Limit Switch
            if (Slide.getCurrentPosition() > threshold){
                Slide.setPower(gamepad2.right_stick_y + (g * Slide.getCurrentPosition()));
            }
            else {
                Slide.setPower(.1+(threshold-Slide.getCurrentPosition())*.001);
            }
                           /**
             * Move robot according to desired axes motions
             * <p>
             * Positive X is forward
             * <p>
             * Positive Y is strafe left
             * <p>
             * Positive Yaw is counter-clockwise
             */
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.y) {
                    imu.resetYaw();
                }
                if (gamepad1.b) {
                    boost = 1.0;
                }
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                Front_Left.setPower(frontLeftPower);
                Back_Left.setPower(backLeftPower);
                Front_Right.setPower(frontRightPower);
                Back_Right.setPower(backRightPower);
                PIVOT.setPower(gamepad2.left_stick_y);

                telemetry.addData("Slide: \n", Slide.getCurrentPosition());
                telemetry.addData("Pivot: \n", PIVOT.getCurrentPosition());
                telemetry.addData("threshold", threshold);


            telemetry.update();

            }



            // Show the elapsed game time and wheel power.
            telemetry.update();
        }

        public double DesiredSlideTicks(int PivotPos) {
            double DesiredSlideTicks = 0;

            double PivotAngle = PivotPos / 80; // Ticks per degree

            if (PivotAngle < 90) {
                //36 is max horizontal extension
                DesiredSlideTicks =  22 / (Math.cos(Math.toRadians(PivotAngle))) * 300.439898; //Ticks per inch
            } else {
                DesiredSlideTicks = -10000;
            }

            return DesiredSlideTicks;



        }
    }



