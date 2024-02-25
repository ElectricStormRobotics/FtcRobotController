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
 * This file contains an minimal example of a how to utilize the neopixel_i2c class to light up
 * an LED strip.
 * This OpMode waits until the opmode has been running for 15 seconds and then turns on the led strip
 * after 20 seconds it changes the color
 * after 25 seconds it change the color again.
 * the number of leds in the strip is defined
 */

@TeleOp(name="Test_LEDv1", group="Linear OpMode")
//@Disabled
public class BasicOpMode_Linear_LED_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime ledtimer = new ElapsedTime();
    private neopixel_i2c LED_strip;
    private boolean led_race=true;

    private boolean led_off = true;
    private boolean end_game = false;
    private int pix_on = 1; //sets all leds on (2 sets every other, 3 sets every 3rd)


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        LED_strip  = hardwareMap.get(neopixel_i2c.class, "led_strip");

        //Assign the RGB order for the led strip
        neopixel_i2c.RGB_order =1; // order 1 means G R B
        int led_countdown = 0;
        int num_pixels = 300; // number of pixels (LEDs) in to work with.
        //LED_strip.doInitialize();
        LED_strip.doInitialized(num_pixels);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        ledtimer.reset();

        boolean doOnce = true;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           int red = 0xFF;
           int blue = 0x00;
           int green = 0x00;

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

    }
    public void waittimer(double time_ms) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();


        while (holdTimer.milliseconds() < time_ms){

        }

    }
}
