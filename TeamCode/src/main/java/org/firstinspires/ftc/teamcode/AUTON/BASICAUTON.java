package org.firstinspires.ftc.teamcode.AUTON;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

import org.firstinspires.ftc.teamcode.MecanumDrive;



    @Config
    @Autonomous(name = "TEST", group = "Autonomous")
    public class BASICAUTON extends LinearOpMode {

        public class slide {
            public int targetPos = 1500;
            private DcMotorEx slide;

            public slide(HardwareMap hardwareMap) {
                slide = hardwareMap.get(DcMotorEx.class, "slide");
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slide.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            public class SlideExtend implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                    }

                    double pos = slide.getCurrentPosition();
                    packet.put("slidePos", pos);

                    if (pos < targetPos) {
                        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slide.setPower(0.8);
                        slide.setTargetPosition(targetPos);
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

            public class SlideDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slide.setPower(-0.8);
                        initialized = true;
                    }

                    double pos = slide.getCurrentPosition();
                    packet.put("liftPos", pos);
                    if (pos > 50.0) {
                        return true;
                    } else {
                        slide.setPower(0);
                        return false;
                    }
                }
            }
            public Action SlideDown(){
                return new SlideDown();
            }
        }

        public class Claw {
            private Servo claw;

            public Claw(HardwareMap hardwareMap) {
                claw = hardwareMap.get(Servo.class, "claw");
            }

            public class CloseClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0.55);
                    return false;
                }
            }
            public Action closeClaw() {
                return new CloseClaw();
            }

            public class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(1.0);
                    return false;
                }
            }
            public Action openClaw() {
                return new OpenClaw();
            }
        }

        @Override
        public void runOpMode() {
            Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            Claw claw = new Claw(hardwareMap);
            slide lift = new slide(hardwareMap);


            // vision here that outputs position
            int visionOutputPosition = 1;

            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                    .lineToYSplineHeading(33, Math.toRadians(0))
                    .waitSeconds(2)
                    .setTangent(Math.toRadians(90))
                    .lineToY(48)
                    .setTangent(Math.toRadians(0))
                    .lineToX(32)
                    .strafeTo(new Vector2d(44.5, 30))
                    .turn(Math.toRadians(180))
                    .lineToX(47.5)
                    .waitSeconds(3);
            TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                    .lineToY(37)
                    .setTangent(Math.toRadians(0))
                    .lineToX(18)
                    .waitSeconds(3)
                    .setTangent(Math.toRadians(0))
                    .lineToXSplineHeading(46, Math.toRadians(180))
                    .waitSeconds(3);
            TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                    .lineToYSplineHeading(33, Math.toRadians(180))
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(46, 30))
                    .waitSeconds(3);
            Action trajectoryActionCloseOut = tab1.fresh()
                    .strafeTo(new Vector2d(48, 12))
                    .build();

            // actions that need to happen on init; for instance, a claw tightening.
            Actions.runBlocking(claw.closeClaw());


            while (!isStopRequested() && !opModeIsActive()) {
                int position = visionOutputPosition;
                telemetry.addData("Position during Init", position);
                telemetry.update();
            }

            int startPosition = visionOutputPosition;
            telemetry.addData("Starting Position", startPosition);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Action trajectoryActionChosen;
            if (startPosition == 1) {
                trajectoryActionChosen = tab1.build();
            } else if (startPosition == 2) {
                trajectoryActionChosen = tab2.build();
            } else {
                trajectoryActionChosen = tab3.build();
            }

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionChosen,

                            lift.SlideExtend(4000),
                            claw.openClaw(),
                            lift.SlideDown(),
                            trajectoryActionCloseOut
                    )
            );
        }
    }

