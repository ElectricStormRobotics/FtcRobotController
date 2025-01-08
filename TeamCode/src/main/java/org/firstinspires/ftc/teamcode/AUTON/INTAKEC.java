package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class INTAKEC {

   private CRServo left_CR;
   private CRServo right_CR;
    public INTAKEC(HardwareMap hardwareMap) {
        left_CR = hardwareMap.get(CRServo.class, "left_CR");
        right_CR = hardwareMap.get(CRServo.class, "right_CR");
        left_CR.setDirection(CRServo.Direction.REVERSE);
    }


    public class IN implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            right_CR.setPower(1);
            left_CR.setPower(1);
            return true;
        }
    }
    public class OUT implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            right_CR.setPower(-1);
            left_CR.setPower(-1);
            return true;
        }
    }
    public class STOP implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            right_CR.setPower(0);
            left_CR.setPower(0);
            return true;
        }
    }


}
