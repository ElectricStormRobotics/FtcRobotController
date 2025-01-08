package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
//@Disabled
public class PIVOTC {
    private DcMotor Pivot;

    public static double PIVOT_POWER = 1;

        public PIVOTC(HardwareMap hardwareMap) {
            Pivot = hardwareMap.get(DcMotor.class, "PIVOT");
            Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        }


        public class movePivotAction implements Action {
            private boolean initialized = true;
            private final int targetPos;

            public movePivotAction(int targetPos) {this.targetPos = targetPos;}

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (initialized) {
                    movePIVOT(targetPos);
                    initialized = false;
                }

                return (Math.abs(targetPos - Pivot.getCurrentPosition()) > 10);
            }
        }

        public Action dMovePivot(int targetpos) {
        return new movePivotAction(targetpos);
        }
        void movePIVOT(int targetpos) {
            Pivot.setTargetPosition(targetpos);
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot.setPower(PIVOT_POWER);
        }
}

