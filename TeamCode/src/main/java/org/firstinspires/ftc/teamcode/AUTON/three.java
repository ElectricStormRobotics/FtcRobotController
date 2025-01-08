package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

import java.util.Arrays;


@Config
    @Autonomous(name = "threeSpec", group = "Autonomous")
@Disabled
    public class three extends LinearOpMode {
        public class PIVOT {
            private DcMotor Pivot;
            int targetPosUp = 1500;
            int targetPosDown = -4000;
            public PIVOT(HardwareMap hardwareMap) {
                Pivot = hardwareMap.get(DcMotor.class, "PIVOT");
                Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Pivot.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            public class PivotDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                    }

                    double pos = Pivot.getCurrentPosition();
                    packet.put("pivotPos", pos);

                    if (pos < targetPosDown) {
                        Pivot.setTargetPosition(targetPosDown);
                        Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Pivot.setPower(0.8);

                        return true;
                    } else {
                        Pivot.setPower(0);
                        return false;
                    }
                }
            }
            public Action PivotDown() {
                return new PivotDown();
            }

            public Action PivotDown(int targetposPivot1) {
                targetPosDown = targetposPivot1;
                return new PivotDown();
            }

            public class PivotUp implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                    }

                    double pos = Pivot.getCurrentPosition();
                    packet.put("pivotPos", pos);

                    if (pos > targetPosUp) {
                        Pivot.setTargetPosition(targetPosUp);
                        Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Pivot.setPower(0.9);

                        return true;
                    } else {
                        Pivot.setPower(0);
                        return false;
                    }
                }
            }
            public Action PivotUp() {
                return new PivotUp();
            }

            public Action PivotUp(int targetposPivot) {
                targetPosUp = targetposPivot;
                return new PivotUp();
            }

        }
    public class slide {
        public slide(HardwareMap hardwareMap) {
            Slide = hardwareMap.get(DcMotor.class, "slide");
            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Slide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public int targetPos = 1500;
        private DcMotor Slide;



        public class SlideExtend implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double pos = Slide.getCurrentPosition();
                packet.put("slidePos", pos);

                if (pos > targetPos) {
                    Slide.setTargetPosition(targetPos);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);

                    return true;
                } else {
                    Slide.setPower(0);
                    return false;
                }
            }
        }

        public Action SlideExtend() {
            return new SlideExtend();
        }

        public Action SlideExtend(int targetposOut) {
            targetPos = targetposOut;
            return new SlideExtend();
        }

        public class Slide0 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }

                double pos = Slide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -50) {
                    Slide.setTargetPosition(0);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    return true;
                } else {
                    Slide.setPower(0);
                    return false;
                }
            }
        }
        public Action Slide0() {
            return new Slide0();
        }
        public class SlideUnExtend implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }

                double pos = Slide.getCurrentPosition();

                packet.put("liftPos", pos);
                if (pos < 0) {
                    Slide.setTargetPosition(0);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    return true;
                } else {
                    Slide.setPower(0);
                    return false;
                }
            }
        }
        public Action SlideUnExtend() {
            return new SlideUnExtend();
        }
        public class SlideScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }

                double pos = Slide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -50) {
                    Slide.setTargetPosition(-2000);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    return true;
                } else {
                    Slide.setPower(0);
                    return false;
                }
            }
        }
        public Action SlideScore() {
            return new SlideScore();
        }

    }

    public class Elbow {
        private Servo elbow;

        public Elbow(HardwareMap hardwareMap) {
            elbow = hardwareMap.get(Servo.class, "elbow");
        }

        public class elbowFWD1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.45);
                return false;
            }
        }
        public Action elbowFWD1() {
            return new elbowFWD1();
        }

        public class elbowScore1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.0);

                return false;
            }
        }
        public Action elbowScore1() {
            return new elbowScore1();
        }
        public class elbowDown1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.45);

                return false;
            }
        }
        public Action elbowDown1() {
            return new elbowDown1();
        }
        public class elbowScoreSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.2);

                return false;
            }
        }
        public Action elbowScoreSpecimen() {
            return new elbowScoreSpecimen();
        }
    }
    public class Wrist {
        private Servo Wrist;

        public Wrist(HardwareMap hardwareMap) {
            Wrist = hardwareMap.get(Servo.class, "wrist");
            Wrist.setDirection(Servo.Direction.REVERSE);
        }

        public class elbowFWD2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.45);
                return false;
            }
        }
        public Action elbowFWD2() {
            return new elbowFWD2();
        }

        public class elbowScore2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.0);

                return false;
            }
        }
        public Action elbowScore2() {
            return new elbowScore2();
        }
        public class elbowDown2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.45);

                return false;
            }
        }
        public Action elbowDown2() {
            return new elbowDown2();
        }
        public class wristScoreSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Wrist.setPosition(0.2);

                return false;
            }
        }
        public Action wristScoreSpecimen() {
            return new wristScoreSpecimen();
        }
    }

    public class IntakeL {
        private CRServo left_CR;

        public IntakeL(HardwareMap hardwareMap) {
            left_CR = hardwareMap.get(CRServo.class, "left_CR");
            left_CR.setDirection(CRServo.Direction.REVERSE);
        }

        public class IN1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_CR.setPower(1);
                return false;
            }
        }
        public Action IN1() {
            return new IN1();
        }

        public class OUT1 implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }
                left_CR.setPower(-1);
                return false;
            }
        }
        public Action OUT1() {
            return new OUT1();
        }

        public class STOP1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_CR.setPower(0);
                return false;
            }
        }
        public Action STOP1() {
            return new STOP1();
        }
    }

    public class IntakeR {
        private CRServo right_CR;

        public IntakeR(HardwareMap hardwareMap) {
            right_CR = hardwareMap.get(CRServo.class, "right_CR");

        }

        public class IN2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                right_CR.setPower(1);
                return false;
            }
        }
        public Action IN2() {
            return new IN2();
        }

        public class OUT2 implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }
                right_CR.setPower(-1);
                return false;
            }
        }
        public Action OUT2() {
            return new OUT2();
        }
        public class STOP2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                right_CR.setPower(0);
                return false;
            }
        }
        public Action STOP2() {
            return new STOP2();
        }
    }
    public class TWIST {
        private Servo twist;

        public TWIST(HardwareMap hardwareMap) {
            twist = hardwareMap.get(Servo.class, "twist");
            twist.setDirection(Servo.Direction.FORWARD);
        }

        public class zeroFlip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(.32);
                return false;
            }
        }
        public Action zeroFlip() {
            return new zeroFlip();
        }

        public class ninetyDe implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(0.0);
                return false;
            }
        }
        public Action ninetyDe() {
            return new ninetyDe();
        }
    }
        public static double V_MEDIUM = 15;
        public static double R_SLOW = 1.5;

    @Override
        public void runOpMode() {
            Pose2d initialPose = new Pose2d(10.0, -63, Math.toRadians(90));
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
            LIFTC Slide = new LIFTC(hardwareMap);
            PIVOTC pivot = new PIVOTC(hardwareMap);
            Elbow Elbow = new Elbow(hardwareMap);
            Wrist Wrist = new Wrist(hardwareMap);
            TWIST twist = new TWIST(hardwareMap);
            IntakeR right_CR = new IntakeR(hardwareMap);
            IntakeL left_CR = new IntakeL(hardwareMap);

            //Set Velocity Constraint to slow things down
        VelConstraint medium = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(V_MEDIUM),
                new AngularVelConstraint(R_SLOW)
        ));




        // vision here that outputs position

        TrajectoryActionBuilder toChamber = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10, -36.5))
                .waitSeconds(.5);
        //turn around and strafe before we go to push blocks
        TrajectoryActionBuilder toPush = toChamber.fresh()
                .strafeToLinearHeading( new Vector2d(38, -39.5), Math.toRadians(270))
                .setTangent(180);
        //push blocks into observation zone
        TrajectoryActionBuilder pushBlock = toPush.fresh()
                .splineToConstantHeading(new Vector2d(40,-10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(270));
        //drive to scoring position with a picked up specimen
        TrajectoryActionBuilder toChamber2 = pushBlock.fresh()
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(4, -39, Math.toRadians(90)), Math.toRadians(45))
                .strafeTo(new Vector2d(4, -35.5));
        //score the specimen on the highchamber
        TrajectoryActionBuilder pickUp = toChamber2.fresh()
                .splineToLinearHeading(new Pose2d(38, -57, Math.toRadians(0)), Math.toRadians(45));
        //pick up the final specimen from the tape line
        TrajectoryActionBuilder toChamberFinal = pickUp.fresh()
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(2, -37, Math.toRadians(90)), Math.toRadians(45))
                .strafeTo(new Vector2d(2, -35.5));
        //throw ourselves into the corner for a park
        TrajectoryActionBuilder park = toChamberFinal.fresh()
                .setTangent(180)
                .strafeToLinearHeading(new Vector2d(45, -55), Math.toRadians(90));
        Action wait = toChamber.fresh()
                .waitSeconds(.5)
                .build();
        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());







        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }



        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(Slide.dMoveSlide(-1500), pivot.dMovePivot(-3700), Elbow.elbowFWD1(), Wrist.elbowFWD2(), toChamber.build())



                        )
        );
    }
}
