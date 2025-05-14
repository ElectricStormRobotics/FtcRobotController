package pedroPathing.AUTONSUBSYSTEMS;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class INTAKESubsystem {

    private CRServo left_CR;
    private CRServo right_CR;

    public INTAKESubsystem(final HardwareMap hMap) {
        left_CR = hMap.get(CRServo.class, "left_CR");
        right_CR = hMap.get(CRServo.class, "right_CR");
        left_CR.setDirection(CRServo.Direction.REVERSE);
    }

    public void IN() {
            right_CR.setPower(1);
            left_CR.setPower(1);
        }
   public void OUT() {
            right_CR.setPower(-1);
            left_CR.setPower(-1);
        }

   public void STOP() {
            right_CR.setPower(0);
            left_CR.setPower(0);
        }



}
