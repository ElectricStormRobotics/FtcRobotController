package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
//@Disabled
public class LIFTC {
    private DcMotor Slide;

    public static double SLIDE_POWER = .8;

        public LIFTC(HardwareMap hardwareMap) {
            Slide = hardwareMap.get(DcMotor.class, "slide");
            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Slide.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public class moveSlideAction implements Action {
            private boolean initialized = true;
            private final int targetPos;

            public moveSlideAction(int targetPos) {this.targetPos = targetPos;}

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (initialized) {
                    moveSlide(targetPos);
                    initialized = false;
                }

                return (Math.abs(targetPos - Slide.getCurrentPosition()) > 10);
            }
        }

        public Action dMoveSlide(int targetpos) {
        return new moveSlideAction(targetpos);
        }
        void moveSlide(int targetpos) {
            Slide.setTargetPosition(targetpos);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(SLIDE_POWER);
        }
}

