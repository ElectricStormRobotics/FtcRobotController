package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.nio.ByteOrder;

/*
 * Driver for Adafruit's NeoDriver I2C to NeoPixel Driver Board Stemma QT
 * https://learn.adafruit.com/adafruit-neodriver-i2c-to-neopixel-driver/overview
 * Created by Joel Cardo Coach/Mentor of CPU Robotics FTC teams 13415 and 6189
 * "Its nice to be important but its more important to be nice" - Dwayne The Rock Johnson
 * based upon Driver for Adafruit's MCP9808 temperature sensor
 * Created by Dryw Wade
 *
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cDeviceType
@DeviceProperties(name = "WS2811 PxDriver", description = "LED Pixel WS2811 Driver from Adafruit", xmlTag = "AdafruitNPDriver")
public class neopixel_i2c extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    // More settings are available on the sensor, but not included here. Could be added later

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //Base address for the NeoPixel module is 0x0E according to the NeoPixel sheet
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);
    public static int RGB_order = 0; // RGB, 1 = GRB



    public neopixel_i2c(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Write Method that writes a strip of LEDs to a specific RGB color
    // need to provide the total number of pixels that are part of the strip
    // Max number of LEDs that can be supported by this method is 85
    protected void writeLEDstrip(int num_pixels, int red, int green, int blue)
    {
        if (red > 255) {
            red = 255; //forces the max value to fit in a byte
        }
        if (green > 255) {
            green = 255; //forces the max value to fit in a byte
        }
        if (blue > 255) {
            blue = 255; //forces the max value to fit in a byte
        }
        // save original colors so the color order can be swapped based on RGB order of LED strip
        int orig_red = red;
        int orig_green = green;
        int orig_blue = blue;

        switch(RGB_order) {
            case 0: //RGB
                red = orig_red;
                green = orig_green;
                blue = orig_blue;
                break;
            case 1: //GRB
                red = orig_green;
                green = orig_red;
                blue = orig_blue;
                break;
            case 2: //BRG
                red = orig_blue;
                green = orig_red;
                blue = orig_green;
                break;
            case 3: //BGR
                red = orig_blue;
                green = orig_green;
                blue = orig_red;
                break;
            default:
                red = orig_red;
                green = orig_green;
                blue = orig_blue;
        }

        int buff_size = 3 * num_pixels; //(3 bytes per pixel)

        //write the size of the pixel buffer into 16bits
        byte[] buff_len = {0x03,(byte)(buff_size/256),(byte)(buff_size%256)};


        //write the size of the pixel buffer to the neopixel buff register
        deviceClient.write(0x0e,buff_len);
        int rounds = num_pixels/5+1;

        for (int j=1; j<(rounds); j++) {
            //loop through writing 5 LEDs at a time
            //because the neopixel only has enough buffer space for 9 LEDs in a single i2c message
            //write the first set of 5 starting with address 0x00
            //increment by 0x0F each time
            //If the number of pixels in this round to write is less than 5
            //make the array size the exact number of pixels in the round
            if (j*5 >= num_pixels){
                byte[] pixel = new byte[3+((num_pixels%5)*3)];
                pixel[0] = 0x04;
                pixel[1] = (byte) ((j-1)*5*3/256);
                pixel[2] = (byte) (((j-1) * 0x0F)%256);

                //create loop that writes RGB values for the remaining LEDs
                for (int i = 0; i < (num_pixels%5); i++) {
                    pixel[3 + i * 3] = (byte) red; // R
                    pixel[4 + i * 3] = (byte) green; // G
                    pixel[5 + i * 3] = (byte) blue; // B
                }
                deviceClient.write(0x0e,pixel);
                //this.writeShow();
            }
            else {
                byte[] pixel = new byte[3 + (5 * 3)];
                pixel[0] = 0x04;
                pixel[1] = (byte) ((j-1)*5*3/256);
                pixel[2] = (byte) (((j-1) * 0x0F)%256);

                //create loop that writes RGB values for the group of 5 LEDs
                for (int i = 0; i < 5; i++) {
                    pixel[3 + i * 3] = (byte) red; // R
                    pixel[4 + i * 3] = (byte) green; // G
                    pixel[5 + i * 3] = (byte) blue; // B
                }
                deviceClient.write(0x0e,pixel);
                //this.writeShow();
            }

        }

        //each pixel is 3 bytes for its addressing
        this.writeShow();
    }

    //Writes a single LED at a specified position (pixPos) in the strip to a color.
    //Leaves the rest of the strip at the color it was.
    //Need to pass in the total number of LEDs in the strip
    protected void writeSingleLED(int pixpos, int numpixels, int red, int green, int blue)
    {
        //pixpos is the pixel position (1 based) off the LED to be set to RGB
        //convert pixpos to 0 based
        pixpos--;
        if (red > 255) {
            red = 255; //forces the max value to fit in a byte
        }
        if (green > 255) {
            green = 255; //forces the max value to fit in a byte
        }
        if (blue > 255) {
            blue = 255; //forces the max value to fit in a byte
        }

        // save original colors so the color order can be swapped based on RGB order of LED strip
        int orig_red = red;
        int orig_green = green;
        int orig_blue = blue;

        switch(RGB_order) {
            case 0: //RGB
                red = orig_red;
                green = orig_green;
                blue = orig_blue;
                break;
            case 1: //GRB
                red = orig_green;
                green = orig_red;
                blue = orig_blue;
                break;
            case 2: //BRG
                red = orig_blue;
                green = orig_red;
                blue = orig_green;
                break;
            case 3: //BGR
                red = orig_blue;
                green = orig_green;
                blue = orig_red;
                break;
            default:
                red = orig_red;
                green = orig_green;
                blue = orig_blue;
        }

        int buff_size = 3 * numpixels; //(3 bytes per pixel)
        //write the size of the pixel buffer into 16bits
        byte[] buff_len = {0x03,(byte)(buff_size/256),(byte)(buff_size%256)};

        //write the size of the pixel buffer to the neopixel buff register
        deviceClient.write(0x0e,buff_len);


        byte[] pixel = new byte[6];
        pixel[0] = 0x04;
        pixel[1] = 0x00;
        pixel[2] = (byte) (pixpos*3);
        pixel[1] = (byte) (pixpos*3/256);
        pixel[2] = (byte) ((pixpos*3)%256);

        pixel[3] = (byte) red; // R
        pixel[4] = (byte) green; // G
        pixel[5] = (byte) blue; // B

        deviceClient.write(0x0e,pixel);
        this.writeShow();
    }

    //Method created for debug/development purposes
    //left in for future debug if needed.
    protected void writeLED()
    {
        byte[] pixel= new byte[9];
        pixel[0] = 0x04; //Function register for data buffer for LED array
        pixel[1] = 0x00; // defines lower byte of the start address of the LED array to write
        pixel[2] = 0x00; // defines upper byte of the start address of the LED array to write
        pixel[3] = 0x0F; // R
        pixel[4] = 0x00; // G
        pixel[5] = 0x00; // B
        pixel[6] = 0x00; // R
        pixel[7] = 0x0F; // G
        pixel[8] = 0x00; // B
        //byte[] show = {0x0e,0x05}; // 0x0e Base register, 0x05 SHOW (sends command to write the data

/*        byte[] pixel = {0x0e,0x04,
                0x0,0x0,
                0xF,0x0,0x0};*/
        //byte[] pin = {0x0e,0x01,0x0f};
        // byte[] speed = {0x0e,0x02,0x00}; // speed is 400 khz
        //byte[] buff_len = {0x0e,0x03,0x00,0x03}; // buff length is 3 bytes (little endian order)
        //deviceClient.write(pin);

        //deviceClient.write(speed);
        //deviceClient.write(buff_len);
        //deviceClient.write(show);
        deviceClient.write(0x0e,pixel);
        //deviceClient.write(show);
    }

    protected void writePin()
    {
        byte[] pin = {0x01,0x0f};
        deviceClient.write(0x0e, pin);
    }

    protected void writeBuff()
    {
        byte[] buff_len = {0x03,0x00,0x06}; // buff length is 3 bytes (little endian order)
        deviceClient.write(0x0e, buff_len);
    }

    protected void writeShow()
    {
        byte[] show = {0x05}; // 0x0e Base register, 0x05 SHOW (sends command to write the data
        deviceClient.write(0x0e, show);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        //establish the neopixel pin
        this.writePin();
        int red = 0;
        int green = 0;
        int blue = 0;
        //sets a strip of at most 30 leds to off
        this.writeLEDstrip(31,red, green, blue);
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName()
    {
        return "Adafruit NeoDriver I2C to NeoPixel Driver Seesaw";
    }

}
