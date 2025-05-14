package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.AUTONSUBSYSTEMS.ARMServoSubsystem;
import pedroPathing.AUTONSUBSYSTEMS.INTAKESubsystem;
import pedroPathing.AUTONSUBSYSTEMS.LiftSubsystem;
import pedroPathing.AUTONSUBSYSTEMS.PIVOTSubsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 Specimens", group = "1Autonomine")
public class SpecimenAuto4 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean timer;

    private PIVOTSubsystem PIVOT;
    private LiftSubsystem lift;
    private ARMServoSubsystem ARM;
    private INTAKESubsystem Intake;


    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int count = 1;
    //Line 1
    private final Pose startPose = new Pose(8, 63);  // Starting position
    private final Pose scorePose = new Pose(36, 63); // Scoring position

    // Line 2
    private final Pose push1HalfcontrolPose = new Pose(25.774, 44.896); // control pt 1
    private final Pose push1HalfcontrolPose2 = new Pose(38.411, 31.926); // control pt 2
    private final Pose push1Half = new Pose(47.058, 34.587); //End at like halfway past the sub on the way to push block 1
    // Line 3
    private final Pose push2ndhalfcontrolPose = new Pose(69.339, 32.430); //control pt1
    private final Pose push2ndhalfcontrolpose2 = new Pose(70.670, 33.284); //control pt2
    private final Pose push2ndhalf = new Pose(26.471, 25.113); //End in human player station with one block pushed

    // Line 4
    private final Pose push2blkcontrolPose = new Pose(66.845, 24.776);//control pt 1
    private final Pose push2blkcontrolPose2 = new Pose(71.961, 11); //control pt2
    private final Pose push2blk = new Pose(28.85, 13.303); //End in Human Player station with 2nd block pushed

    //Line 5
    private final Pose pickUp1 = new Pose(11, 47.746); // End facing the pickup spot ready to pick up

    // Line 6
    private final Pose scorecontrolPose = new Pose(-10, 85.651); //control pt 1
    private final Pose score2 = new Pose(34.5, 68.977); //Score the second specimen

    // Line 7
    private final Pose pickUp3controlPose = new Pose(0, 77.986); // control pt1
    private final Pose pickUp3 = new Pose(8.3, 50.946); // End facing the pickup spot ready to pick up

    // Line 8
    private final Pose score3controlPose = new Pose(1, 100); // control pt1
    private final Pose score3 = new Pose(34.5, 67.824); //Score third Specimen

    //Line 9
    private final Pose pickUp4controlPose = new Pose(1.164, 71.834); //control pt1
    private final Pose pickUp4 = new Pose(8.2, 50.946); // End facing the pickup spot ready to pick up

    // Line 10
    private final Pose score4controlPose = new Pose(1, 100); //control pt1
    private final Pose score4 = new Pose(33.576, 63.838); // Score final specimen epically

    // Line 11
    private final Pose parkControlPose = new Pose(6.319, 75.824); //control pt1
    private final Pose parkPose = new Pose(10.129, 11.296); // Park epically securing 43 pts
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain push1half, push2, grabPickUp2, scorePickup2, grabPickUp3, scorePickup3, grabPickUp4, scorePickup4, parkWill;
    public void buildPathswill() {
        //Score preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        push1half = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(push1HalfcontrolPose),
                                new Point(push1HalfcontrolPose2),
                                new Point(push1Half)


                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(push1Half),
                                new Point(push2ndhalfcontrolPose),
                                new Point(push2ndhalfcontrolpose2),
                                new Point(push2ndhalf)


                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(push2ndhalf),
                                new Point(push2blkcontrolPose),
                                new Point(push2blkcontrolPose2),
                                new Point(push2blk)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(push2blk),
                                new Point(pickUp1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();



        grabPickUp2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(push2blk),
                                new Point(pickUp1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickUp1),
                                new Point(scorecontrolPose),
                                new Point(score2)
                                )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();
        grabPickUp3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(score2),
                                new Point(pickUp3controlPose),
                                new Point(pickUp3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickUp3),
                                new Point(score3controlPose),
                                new Point(score3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();
        grabPickUp4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(score3),
                                new Point(pickUp4controlPose),
                                new Point(pickUp4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickUp4),
                                new Point(score4controlPose),
                                new Point(score4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();
        parkWill = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(score4),
                                new Point(parkControlPose),
                                new Point(parkPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

    }
    /*
    public void buildPaths() {
        // Path for scoring preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        // Path chains for picking up and scoring samples
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Curved path for parking
        park = new Path(
                new BezierCurve(
                new Point(scorePose),
                new Point(parkControlPose),
                new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: { // Move arm to scoring positions
                ARM.fwd();
                lift.moveSlide(-1500);
                PIVOT.movePIVOT(-3700);
                setPathState(1);
                break;
            }
            case 1: { // Move from start to scoring position
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            }
            case 2 : { // Rotate elbow/wrist up to lock specimen
                if (!follower.isBusy()) {
                    ARM.UP();
                    setPathState(10);
                }
                break;
             }

            case 10: { // Adjust arm and pushblock
                if (pathTimer.getElapsedTimeSeconds() > 0.4) { //Wait then adjust PIVOT and push block 1
                    lift.moveSlide(-450);
                    PIVOT.movePIVOT(-1400);
                    follower.followPath(push1half);
                    setPathState(21);
                }
                break;
            }
            case 11: {

                break;
            }

            case 21: { // Move arms to pick up position and drive to Pick up position
                if (!follower.isBusy()) {
                    setPathState(31);
                    timer = true;
                }
                break;
            }

            case 31: { // Increase count variable (starts at 1) and cycle through pick up  sequence
                if (!follower.isBusy()) {
                    if (timer) {
                        count++;
                        pathTimer.resetTimer();
                        timer = false;
                    }

                    ARM.DOWN();
                    ARM.ninetyDe();
                    Intake.IN();
                    if (pathTimer.getElapsedTimeSeconds() > .75) { // Move arm down to intake Specimen
                        PIVOT.movePIVOT(-500);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1) { // Wait half a second to make sure specimen is in
                        setPathState(32);
                    }
                }
                break;
            }
            case 32: { // Move slides to scoring position
                if (!PIVOT.isBusyPIVOT()) {
                    lift.moveSlide(-1500);
                    PIVOT.movePIVOT(-3700);
                    ARM.zeroFlip();
                    ARM.fwd();
                    setPathState(40);
                }
                break;
            }

            case 40: { // Pick between the count variables to decide which path to take to score the specimen
                if (count == 2) { // Drive to score second Specimen
                    follower.followPath(scorePickup2);
                    setPathState(41);
                }
                if (count == 3) { // Drive to score third Specimen
                   setPathState(60);
                }
                if (count == 4) { // Drive to score fourth Specimen
                    setPathState(80);
                }
                break;
            }

            case 41: { // Move arm up and score second specimen
                if (!follower.isBusy()){
                    Intake.STOP();
                    ARM.UP();
                    setPathState(50);
                }
                break;
             }

            case 50: { // Move chassis to pickup position and arm pickup orientation
                if (pathTimer.getElapsedTimeSeconds() > .25) { // Chassis to pickup position
                    follower.followPath(grabPickUp3, true);
                }
                if (pathTimer.getElapsedTimeSeconds() > .6) { // Arm to pickup orientation then jump back to case 31
                    ARM.ninetyDe();
                    Intake.IN();
                    ARM.DOWN();
                    lift.moveSlide(-450);
                    PIVOT.movePIVOT(-1400);
                    setPathState(31);
                    timer = true;
                }
                break;
            }

            case 60: { // Move chassis to scoring position
                    follower.followPath(scorePickup3);
                    setPathState(61);

                break;
            }
            case 61: { // Score third specimen
                if (!follower.isBusy()) {
                    Intake.STOP();
                    ARM.UP();
                    setPathState(70);
                }
                break;
            }
            case 70: { // Move to pick up location
                if (pathTimer.getElapsedTimeSeconds() > .25) { // Chassis to pickup position
                    follower.followPath(grabPickUp4, true);
                }
                if (pathTimer.getElapsedTimeSeconds() > .6) { // Arm to pickup orientation then jump back to case 31
                    ARM.ninetyDe();
                    Intake.IN();
                    ARM.DOWN();
                    lift.moveSlide(-450);
                    PIVOT.movePIVOT(-1400);
                    setPathState(31);
                    timer = true;
                }
                break;
            }
            case 80: { // Move chassis to scoring position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup4);
                    setPathState(81);
                }
                break;
            }
            case 81: { // Score 4th specimen
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    Intake.STOP();
                    ARM.UP();

                    setPathState(90);
                }
                break;
            }
            case 90: { // Move slides and PIVOT down delayed
                if (pathTimer.getElapsedTimeSeconds() > .4) {
                    lift.moveSlide(-100);
                    PIVOT.movePIVOT(-1000);
                }
                break;
            }

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        PIVOT = new PIVOTSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        ARM = new ARMServoSubsystem(hardwareMap);
        Intake = new INTAKESubsystem(hardwareMap);
        ARM.zeroFlip();
        ARM.UP();
        buildPathswill();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("count", count);
        telemetry.update();
    }
}
