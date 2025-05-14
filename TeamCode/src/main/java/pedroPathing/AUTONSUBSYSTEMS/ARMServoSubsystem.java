package pedroPathing.AUTONSUBSYSTEMS;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ARMServoSubsystem extends SubsystemBase {

    private final Servo wrist;
    private final Servo elbow;
    private final Servo twist;
    double down = 0.6;
    double fwd = .45;
    double up = 0.0;
    double sample = 0.25;
    double zeroFlip = 0.32;
    double ninetyDe = 0.0;
    double one80 = .65;
    double two70 = 1.0;


    public ARMServoSubsystem(final HardwareMap hMap) {
        wrist = hMap.get(Servo.class, "wrist");
        elbow = hMap.get(Servo.class, "elbow");
        twist = hMap.get(Servo.class, "twist");
        elbow.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
    }


    public void UP() {
        elbow.setPosition(up);
        wrist.setPosition(up);
    }

    public void DOWN() {
        elbow.setPosition(down);
        wrist.setPosition(down);
    }

    public void fwd() {
        elbow.setPosition(fwd);
        wrist.setPosition(fwd);
    }

    public void ElbowSmple() {
        elbow.setPosition(sample);
        wrist.setPosition(sample);
    }

    public void zeroFlip() {
        twist.setPosition(zeroFlip);
    }

    public void ninetyDe() {
        twist.setPosition(ninetyDe);
    }

    public void one80() {
        twist.setPosition(one80);
    }

    public void two70() {
        twist.setPosition(two70);
    }

    public void autoPic(double pic){

        twist.setPosition(pic);


    }

}
