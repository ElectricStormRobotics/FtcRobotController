package pedroPathing.AUTONSUBSYSTEMS;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase{
    private DcMotor Slide = null;  //  Used to control both of the slides
    public static double SLIDE_POWER = 1;

    public LiftSubsystem(final HardwareMap hMap) {
        Slide = hMap.get(DcMotor.class, "slide");
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Slide.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void moveSlide(int targetpos) {
        Slide.setTargetPosition(targetpos);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(SLIDE_POWER);
    }
    public boolean isBusyLift() {
        return Slide.isBusy();
    }


}
