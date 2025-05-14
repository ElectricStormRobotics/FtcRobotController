package pedroPathing.AUTONSUBSYSTEMS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class ARMC {
    private Servo elbow; // Brings Arm attachment up and down
    private Servo wrist; // Brings Arm attachment up and down
    private Servo twist; // Twists Intake

    double down = 0.6;
    double fwd = .45;
    double up = 0.0;
    double sample = 0.25;
    double zeroFlip = 0.32;
    double ninetyDe = 0.0;
    double one80 = .65;
    double two70 = 1.0;
    public ARMC(HardwareMap hardwareMap) {

        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        twist = hardwareMap.get(Servo.class, "twist");
        elbow.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);

    }

    public class ElbowUP {

        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(up);
            wrist.setPosition(up);
            return true;
        }
    }

    public class ElbowDOWN  {

        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(down);
            wrist.setPosition(down);
            return true;
        }
    }
    public class ElbowFWD  {

        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(fwd);
            wrist.setPosition(fwd);
            return true;
        }
    }
    public class Elbowsmple  {

        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(sample);
            wrist.setPosition(sample);
            return true;
        }
    }
    public class zeroFlip  {

        public boolean run(@NonNull TelemetryPacket packet) {
            twist.setPosition(zeroFlip);
            return true;
        }
    }
    public class ninety  {

        public boolean run(@NonNull TelemetryPacket packet) {
            twist.setPosition(ninetyDe);
            return true;
        }
    }
    public class one80  {

        public boolean run(@NonNull TelemetryPacket packet) {
            twist.setPosition(one80);
            return true;
        }
    }
    public class two70  {

        public boolean run(@NonNull TelemetryPacket packet) {
            twist.setPosition(two70);
            return true;
        }
    }
}
