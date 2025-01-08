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

import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


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
@Config
@TeleOp(name="ARM TESTER", group="Linear OpMode")
@Disabled
public class ARM extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private ElapsedTime ledtimer = new ElapsedTime();

    private DcMotor Slide = null;  //  Used to control both of the slides
    private DcMotor PIVOT = null; // Lead Screw manipulator


    private Servo elbow = null; // Brings Arm attachment up and down
    private Servo wrist = null; // Brings Arm attachment up and down

    private Servo twist = null; // Twists Intake

    private CRServo right_CR; //Right Intake wheel
    private CRServo left_CR; //Left Intake wheel


    double elbowdown = 0.6;
    double elbowup = 0.0;
    double zeroFlip = 0.32;
    double ninetyDe = 0.0;
    double one80 = .65;
    double two70 = 1.0;

    double maxExten = -6000;
    boolean armup =  false;
    private PIDController SlideController;
    public static double pS = 0.01, iS = 0, dS = 0.000005;
    public static double f = 0;
    private PIDController armController;
    public static double pA = 0.005, iA = 0, dA = 0;
    public static int TargetSlide = 0;
    private int SLIDE_RIGHTSTICK_BUTTON = -1400;
    private int SLIDE_LEFTSTICK_BUTTON = -1500;
    public static int TargetArm = 0;
    private int ARM_RIGHTSTICK_BUTTON = -400;
    private int ARM_LEFTSTICK_BUTTON = -3700;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        Slide = hardwareMap.get(DcMotor.class, "slide");
        PIVOT = hardwareMap.get(DcMotor.class, "PIVOT");
        left_CR = hardwareMap.get(CRServo.class, "left_CR");
        right_CR = hardwareMap.get(CRServo.class, "right_CR");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        twist = hardwareMap.get(Servo.class, "twist");

        armController = new PIDController(pA, iA, dA);
        SlideController = new PIDController(pS, iS, dS);

        // LED_strip  = hardwareMap.get(neopixel_i2c.class, "led_strip");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
      //  Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // PIVOT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIVOT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        left_CR.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        waitForStart();
        elbow.setPosition(0);
        wrist.setPosition(0);
        twist.setPosition(zeroFlip);
        runtime.reset();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armController.setPID(pA, iA, dA);
            SlideController.setPID(pS, iS, dS);
            int currentPIVOTPOS = PIVOT.getCurrentPosition();
            int currentSlidePOS = Slide.getCurrentPosition();
            double PivotAngle = currentPIVOTPOS / 80; // Ticks per degree


            double threshold = 0;
                threshold = -DesiredSlideTicks(currentPIVOTPOS);


            if (gamepad2.y) {
                elbow.setPosition(elbowup);
                wrist.setPosition(elbowup);
            }
            if (gamepad2.a) {
                elbow.setPosition(elbowdown);
                wrist.setPosition(elbowdown);
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

            //SLIDE CONTROLS
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                if (currentSlidePOS > threshold){
                    Slide.setPower(gamepad2.right_stick_y);
                }
                else {
                    Slide.setPower(.1+(threshold-currentSlidePOS)*.001);
                }
                TargetSlide = Slide.getCurrentPosition();
            }
            else {
                double ff = (Math.sin(Math.toRadians(PivotAngle)) * f * currentSlidePOS);
                double SlidePower = SlideController.calculate(currentSlidePOS, TargetSlide);
                Slide.setPower(SlidePower + ff);
            }

            //PIVOT CONTROLS
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                PIVOT.setPower(gamepad2.left_stick_y);
                TargetArm = PIVOT.getCurrentPosition();
            }
            else {
                double ArmPower = armController.calculate(currentPIVOTPOS, TargetArm);
                PIVOT.setPower(ArmPower);
            }
            //PICKUP POSITION
            if(gamepad2.right_stick_button) {
                TargetArm = ARM_RIGHTSTICK_BUTTON;
                TargetSlide = SLIDE_RIGHTSTICK_BUTTON;
            }
            //SCORING POSITION
            if(gamepad2.left_stick_button) {
                TargetArm = ARM_LEFTSTICK_BUTTON;
                TargetSlide = SLIDE_LEFTSTICK_BUTTON;
            }

            if(currentPIVOTPOS < -4500) {
                armup = true;
            }
            else{
                    armup =false;
            }

            //DONE WITH DRIVER 1
            if (!armup) {
                maxExten = -6000;
                elbowup = 0.0;
            }
            if (armup) {
                elbowup = 0.2;
                maxExten = -8000;
            }



                telemetry.addData("Slide: \n", currentSlidePOS);
                telemetry.addData("Pivot: \n", currentPIVOTPOS);
                telemetry.addData("TargetArm", TargetArm);
                telemetry.addData("TargetSlide", TargetSlide);
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



