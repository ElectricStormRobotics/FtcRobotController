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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



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

@TeleOp(name="Test_LED", group="Linear OpMode")
//@Disabled
public class BasicOpMode_Linear_LED_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private neopixel_i2c LED_strip;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        LED_strip  = hardwareMap.get(neopixel_i2c.class, "led_strip");
        //sleep(500);
        //LED_strip.writeLED();
        LED_strip.doInitialize();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean doOnce = true;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           int red = 0xFF;
           int blue = 0x00;
           int green = 0x00;
           int num_pixels = 18; // number of pixels (LEDs) in to work with.
            // Doesn't need to be the whole strip, but needs to be all the LEDs planned to be used


           if(doOnce) {
               LED_strip.writeLEDstrip(num_pixels, red, green, blue);
               doOnce = false;
               // Show the elapsed game time and wheel power.
               telemetry.addData("LED Strip Color", "testing color Red");
               telemetry.update();
               waittimer(100);
           }
            for(int i=0;i<num_pixels;i++){
                LED_strip.writeSingleLED(i,num_pixels,0x00,0x00,0xFF);
                waittimer(25);
                doOnce=false;
            }
            for(int i=0;i<num_pixels;i++){
                LED_strip.writeSingleLED(i,num_pixels,0xFF,0x00,0x00);
                waittimer(25);
            }

            //example code below sets the strip to Green, then Blue, then Yellow, then Pink
            //waits 500 milliseconds
           /* red = 0x00;
            green = 0xFF;
            blue = 0x00;
            LED_strip.writeLEDstrip(num_pixels, red, green, blue);
            // Show the elapsed game time and wheel power.
            telemetry.addData("LED Strip Color", "testing color Green");
            telemetry.update();
            waittimer(500);
            red = 0x00;
            green = 0x00;
            blue = 0xFF;
            LED_strip.writeLEDstrip(num_pixels, red, green, blue);
            // Show the elapsed game time and wheel power.
            telemetry.addData("LED Strip Color", "testing color Blue");
            telemetry.update();
            waittimer(500);
            red = 0xFF;
            green = 0xFF;
            blue = 0x00;
            LED_strip.writeLEDstrip(num_pixels
                    ,red, green, blue);
            // Show the elapsed game time and wheel power.
            telemetry.addData("LED Strip Color", "testing color Yellow");
            telemetry.update();
            waittimer(500);
            red = 0xFF;
            green = 0x14;
            blue = 0x93;
            LED_strip.writeLEDstrip(num_pixels
                    ,red, green, blue);
            // Show the elapsed game time and wheel power.
            telemetry.addData("LED Strip Color", "testing color PINK");
            telemetry.update();
            waittimer(500);*/
        }

    }
    public void waittimer(double time_ms) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();


        while (holdTimer.milliseconds() < time_ms){

        }

    }
}
