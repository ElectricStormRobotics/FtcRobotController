package pedroPathing.AUTONSUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PIVOTSubsystem extends SubsystemBase{
    private DcMotor PIVOT = null;  //  Used to control the PIVOT
    public static double PIVOT_POWER = 1.0;

    public PIVOTSubsystem(final HardwareMap hMap) {
        PIVOT = hMap.get(DcMotor.class, "PIVOT");
        PIVOT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIVOT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIVOT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIVOT.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void movePIVOT(int targetpos) {
        PIVOT.setTargetPosition(targetpos);
        PIVOT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PIVOT.setPower(PIVOT_POWER);
    }
    public boolean isBusyPIVOT() {
        return PIVOT.isBusy();
    }

}
