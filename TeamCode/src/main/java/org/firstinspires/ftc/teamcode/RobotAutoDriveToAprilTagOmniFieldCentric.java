/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */
// HELLO WORLD
@TeleOp(name="Omni Drive To AprilTag Field Centric", group = "Concept")
//@Disabled
public class RobotAutoDriveToAprilTagOmniFieldCentric extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 1; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor Front_Right = null;  //  Used to control the left front drive wheel
    private DcMotor Front_Left = null;  //  Used to control the right front drive wheel
    private DcMotor Back_Left = null;  //  Used to control the left back drive wheel
    private DcMotor Back_Right = null;  //  Used to control the right back drive wheel
    private DcMotor Intake = null; // Used for the intake pulleys
    private DcMotor SlideLeft =null; // First slide motor
    private DcMotor SlideRight =null;// Second slide motor
    private DcMotor Winch = null; // Raises the Robot
    private Servo Wrist =null; // Wrist Servo
    private Servo Bucket =null; //Bucket Servo
    private Servo Hanger = null; //Hanger Servo
    private Servo IntakeLinkage = null; // Runs Linkage for the intake drop down
    private Servo Lens = null; // Moves Lens on and off for Prop and April Tags
    //private DistanceSensor LeftDistance;
    //private DistanceSensor RightDistance;
    //double avgdist = 0;
    private Servo Drone = null; //Drone launcher servo
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    static int DESIRED_TAG_ID = -1;// Choose the tag you want to approach or set to -1 for ANY tag

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    int redTeam = 1;
    int blueTeam = 1;
    int intDirection = 1;  // Into Robot
    int intakeOn = -1;     // Intake Off
    double lwst = 0.8; // How far the intake goes down
    double g = (.00005); // Slide is all the way down
    double intakePwr = 1; // sets the pwr to intake
    double intakeup = 1; // the intake is up
    double boost = .55;
    boolean wristToggle = false;
    boolean intakeToggle = false;



    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // By setting these values to new Gamepad(), they will default to all
        // boolean values as false and all float values as 0



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        Front_Right  = hardwareMap.get(DcMotor.class, "Front_Right");
        Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
        Back_Left  = hardwareMap.get(DcMotor.class, "Back_Left");
        Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        SlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotor.class, "SlideRight");
        Winch = hardwareMap.get(DcMotor.class, "Winch");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        Hanger = hardwareMap.get(Servo.class,"Hanger");
        IntakeLinkage = hardwareMap.get(Servo.class,"IntakeLinkage");
        Lens = hardwareMap.get(Servo.class,"Lens");
        //LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        //RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

       // Rev2mDistanceSensor sensorTimeOfFlightLeft = (Rev2mDistanceSensor) LeftDistance;
       // Rev2mDistanceSensor sensorTimeOfFlightRight = (Rev2mDistanceSensor) RightDistance;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Front_Left.setDirection(DcMotor.Direction.REVERSE);
        Back_Left.setDirection(DcMotor.Direction.REVERSE);
        Front_Right.setDirection(DcMotor.Direction.FORWARD);
        Back_Right.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Winch.setDirection(DcMotorSimple.Direction.FORWARD);
        Wrist.setDirection(Servo.Direction.FORWARD);
        Bucket.setDirection(Servo.Direction.REVERSE);
        Hanger.setDirection(Servo.Direction.FORWARD);
        IntakeLinkage.setDirection(Servo.Direction.FORWARD);
        Lens.setDirection(Servo.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        //If x or b is pressed team will be set to the corresponding color

        while (opModeInInit()) {
            boost = .55;
            Lens.setPosition(0.6);
            Bucket.setPosition(0.075);
            Wrist.setPosition(0.03);
            Hanger.setPosition(0.0);
            IntakeLinkage.setPosition(0.0);
            telemetry.addData("Intake Linkage", IntakeLinkage.getPosition());

            if (gamepad1.x) {
                blueTeam = 1;
                redTeam = 0;
                telemetry.addData("Red ", "Team");
                telemetry.update();
            } else if (gamepad1.b) {
                redTeam = 1;
                blueTeam = 0;
                telemetry.addData("Blue ", "Team");

                telemetry.update();
            } else {
                telemetry.addData("Red: ", redTeam);
                telemetry.addData("Blue: ", blueTeam);

                telemetry.update();
            }
        }
        Drone.setPosition(0);

        // waitForStart();  commented out because it may no longer be needed.  delete at some point

        while (opModeIsActive())
        {
            //avgdist = (RightDistance.getDistance(DistanceUnit.INCH) + LeftDistance.getDistance(DistanceUnit.INCH))/2;
            //telemetry.addData("deviceName", LeftDistance.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", LeftDistance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("rangeleft", String.format("%.01f cm", LeftDistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", LeftDistance.getDistance(DistanceUnit.METER)));
            //telemetry.addData("rangeleft", String.format("%.01f in", LeftDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlightRight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightRight.didTimeoutOccur()));

            //telemetry.addData("deviceName", RightDistance.getDeviceName() );
            //telemetry.addData("range", String.format("%.01f mm", RightDistance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("rangeright", String.format("%.01f cm", RightDistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", RightDistance.getDistance(DistanceUnit.METER)));
            //telemetry.addData("rangeright", String.format("%.01f in", RightDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlightRight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlightRight.didTimeoutOccur()));
            //telemetry.update();


            targetFound = false;
            desiredTag  = null;
            Lens.setPosition(0);
            if (gamepad1.x) {
                intDirection = 1;   // Goes Into Robot
            }
            else if (gamepad1.b) {
                intDirection = -1;   // Goes Out of Robot
            }
            if (intakeOn == 1) {
                Intake.setPower(intakePwr *intDirection);  // Sets the Intake to On
            }
            else {
                Intake.setPower(intakePwr *0); // Turns the Intake off
            }
            //Right Bumper will go to second desired tag below. (Make sure to set desired tag to -1 above)
            //
            if (gamepad1.right_bumper && gamepad1.left_bumper && redTeam == 1) {
                DESIRED_TAG_ID = 5;
                boost = 1;
            }
            else if (gamepad1.left_bumper && !gamepad1.right_bumper && redTeam == 1) {
                DESIRED_TAG_ID = 4;
                boost = 1;
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper && redTeam == 1) {
                DESIRED_TAG_ID = 6;
                boost = 1;
            }

            if (gamepad1.right_bumper && gamepad1.left_bumper && blueTeam == 1) {
                DESIRED_TAG_ID = 2;
                boost = 1;
            }
             else if (gamepad1.left_bumper && !gamepad1.right_bumper && blueTeam == 1){
                DESIRED_TAG_ID = 1;
                boost = 1;
            }
             else if (gamepad1.right_bumper && !gamepad1.left_bumper && blueTeam == 1){
                DESIRED_TAG_ID = 3;
                boost = 1;
            }


             else if (gamepad2.y && Wrist.getPosition()<.3 && SlideLeft.getCurrentPosition() <= -200) {
                Wrist.setPosition(.7);
                Bucket.setPosition(0.3);

            }
             if (gamepad1.x || gamepad1.b)   {
                 intakeOn = 1;
             }
            if (gamepad1.a) {
                intakeOn = -1;
            }

            if (gamepad1.dpad_up && lwst >= .4) {
                lwst = (lwst - 0.025);
            }
            else if (gamepad1.dpad_down && lwst <= 0.8) {
                lwst = (lwst + 0.025);
            }

            if (gamepad2.right_bumper && Bucket.getPosition() > 0.3 && SlideLeft.getCurrentPosition() <= -200) {
                 Bucket.setPosition(0.075);
                 Wrist.setPosition(0.03);
             }
             else if (gamepad2.left_bumper && Bucket.getPosition() < 0.4 && Wrist.getPosition() > .6) {
                 Wrist.setPosition(0.9);
                 Bucket.setPosition(0.7);
             }
             if (gamepad2.x) {

                 intakeup = intakeup*-1;
             }
             if (intakeup < 0){
                 IntakeLinkage.setPosition(0.0);
             }
             else {
                 IntakeLinkage.setPosition(lwst);
             }
   /*          if (avgdist <= .5){
                 telemetry.addLine("UNDER 1 INCHES");
                 boost = .15;
             }
             else {
                 boost = .55;
             }
             while (gamepad1.dpad_up & avgdist > 2) {
                 moveRobot2AprilTag(avgdist, 0,0);
             }
*/
            if (gamepad2.back && gamepad1.back) {
                Drone.setPosition(.25);
            }
             telemetry.update();



            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }


            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">","HOLD Left-Bumper or Right-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("SlideLeft Position", SlideLeft.getCurrentPosition());
                telemetry.addData("SlideRight Position", SlideRight.getCurrentPosition());
                telemetry.addData("Wrist Position", Wrist.getPosition());
                telemetry.addData("Bucket position", Bucket.getPosition());

                telemetry.update();

            }
            else {
                telemetry.addData(">","Drive using joysticks to find valid target\n");

                telemetry.addData("SlideLeft Position", SlideLeft.getCurrentPosition());
                telemetry.addData("SlideRight Position", SlideRight.getCurrentPosition());
                telemetry.addData("Wrist Position", Wrist.getPosition());
                telemetry.addData("Bucket position", Bucket.getPosition());
                telemetry.addData("Intake Linkage", IntakeLinkage.getPosition());
                //telemetry.addData("Distance to Board", (String.format("%.01f cm", (LeftDistance.getDistance(DistanceUnit.CM) + (RightDistance.getDistance(DistanceUnit.CM)))/2)));
                telemetry.update();
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.right_bumper && gamepad1.left_bumper && targetFound)
            {
                boost = 1;
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot2AprilTag(drive, strafe, turn);
            }
            else if (gamepad1.left_bumper && !gamepad1.right_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                boost = 1;
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot2AprilTag(drive, strafe, turn);
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                boost = 1;
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot2AprilTag(drive, strafe, turn);
            }

            else {
                boost = .55;                /**
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
                SlideLeft.setPower(gamepad2.left_stick_y+(g*SlideLeft.getCurrentPosition()));
                SlideRight.setPower(gamepad2.left_stick_y+(g*SlideRight.getCurrentPosition()));

            }
            telemetry.update();
        }
}

    public void waittimer(double time) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
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
    public void moveRobot2AprilTag(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y -yaw;
        double rightFrontPower   =  x + y +yaw;
        double leftBackPower     =  x + y -yaw;
        double rightBackPower    =  x - y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.

        Front_Left.setPower(leftFrontPower*boost);
        Front_Right.setPower(rightFrontPower*boost);
        Back_Left.setPower(leftBackPower*boost);
        Back_Right.setPower(rightBackPower*boost);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                waittimer(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                waittimer(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            waittimer(50);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            waittimer(20);
        }
    }
}

