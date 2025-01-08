package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp
public class FTCLIBSLIDE extends OpMode{
    private PIDController armcontroller;

    public static double p = 0, i = 0, d = 0;
    public static double f =0;

    public static int target = 0;

    private static double ticks_in_degree = 537.7/360;

    private DcMotorEx slide_motor;


    @Override
    public void init() {
        armcontroller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide_motor = hardwareMap.get(DcMotorEx.class, "slide");
        slide_motor.setDirection(DcMotorEx.Direction.REVERSE);
        slide_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        armcontroller.setPID(p, i, d);

        int armPos = slide_motor.getCurrentPosition();

        double pid = armcontroller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid * ff;

        slide_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}