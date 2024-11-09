package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;


@Config
    @Autonomous(name = "TEST", group = "Autonomous")
    public class BASICAUTON extends LinearOpMode {
        public class PIVOT {
            private DcMotor Pivot;
            int targetPos = 1500;
            public PIVOT(HardwareMap hardwareMap) {
                Pivot = hardwareMap.get(DcMotor.class, "PIVOT");
                Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Pivot.setDirection(DcMotorSimple.Direction.FORWARD);
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

                    if (pos > targetPos) {
                        Pivot.setTargetPosition(targetPos);
                        Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Pivot.setPower(0.8);

                        return true;
                    } else {
                        return false;
                    }
                }
            }
            public Action PivotUp() {
                return new PIVOT.PivotUp();
            }

            public Action PivotUp(int targetposPivot) {
                targetPos = targetposPivot;
                return new PIVOT.PivotUp();
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

        public class SlideUnExtend implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    initialized = true;
                }

                double pos = Slide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -50) {
                    Slide.setTargetPosition(-40);
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
    }

        @Override
        public void runOpMode() {
            Pose2d initialPose = new Pose2d(10.0, -63, Math.toRadians(90));
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
            slide lift = new slide(hardwareMap);
            PIVOT pivot = new PIVOT(hardwareMap);
            Elbow Elbow = new Elbow(hardwareMap);
            Wrist Wrist = new Wrist(hardwareMap);


            // vision here that outputs position


            TrajectoryActionBuilder toChamber = drive.actionBuilder(initialPose)
                    .lineToY(-34)
                    .waitSeconds(.5);
            Action toHP = toChamber.fresh()

                    .strafeTo(new Vector2d(35,-37))
                    .splineToConstantHeading(new Vector2d(40,-10), Math.toRadians(90))
                    .waitSeconds(.5)
                    .setTangent(0)
                    .lineToXConstantHeading(50)
                    .waitSeconds(.5)
                    .lineToYConstantHeading(-55)
                    .lineToYConstantHeading(-55)

                    .build();

            Action wait = toChamber.fresh()
                    .waitSeconds(1)
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
                            new ParallelAction(lift.SlideExtend(-1500), pivot.PivotUp(-3700), Elbow.elbowFWD1(), Wrist.elbowFWD2()),
                            toChamber.build(),
                            Elbow.elbowScore1(),
                            Wrist.elbowScore2(),
                            wait,
                            lift.SlideUnExtend(),
                            toHP,
                            new ParallelAction(Elbow.elbowFWD1(), Wrist.elbowFWD2()),
                            wait
                    )
            );
        }
    }

