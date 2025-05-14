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

@Autonomous(name = "2 Specimens", group = "1Autonomine")
public class SpecimenAuto2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PIVOTSubsystem PIVOT;
    private LiftSubsystem lift;
    private ARMServoSubsystem ARM;
    private INTAKESubsystem Intake;


    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private final Pose startPose = new Pose(8, 63);  // Starting position
    private final Pose scorePose = new Pose(37, 63); // Scoring position

    private final Pose pickup1controlPose = new Pose(19.455, 62.356); // First sample pickup
    private final Pose pickup1controlPose2 = new Pose(19.788, 89.176); // Second sample pickup
    private final Pose pickup1Pose = new Pose(10, 50.746); // Third sample pickup

    private final Pose score2controlPose = new Pose(0, 90.014);    // Parking position
    private final Pose score2Pose = new Pose(38.079, 70.171); // Control point for curved path

    private final Pose parkControlPose = new Pose(15.797, 68.841);
    private final Pose parkControlPose2 = new Pose (21.617, 68.176);

    private final Pose parkPose = new Pose(11.972, 14.467);
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain pickUp1, score2, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, parkWill;
    public void buildPathswill() {
        //Score preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

        //Pick up the second Specimen
        pickUp1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(pickup1controlPose),
                        new Point(pickup1controlPose2),
                        new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickup1Pose),
                        new Point(score2controlPose),
                        new Point(score2Pose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        //Pick up the second Specimen
        parkWill = follower.pathBuilder()
                .addPath(
                new BezierCurve(
                        new Point(score2Pose),
                        new Point(parkControlPose),
                        new Point(parkControlPose2),
                        new Point(parkPose)
                )
        )
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
            case 0: // Move arm items to scoring positions
                ARM.fwd();
                lift.moveSlide(-1500);
                PIVOT.movePIVOT(-3700);
                setPathState(1);
                break;

            case 1: { // Move from start to scoring position
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            }
            case 2: { // Score
                if (!follower.isBusy()) {
                    ARM.UP();
                    setPathState(3);
                }
                break;
            }
            case 3: {
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(pickUp1, true);

                }
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    ARM.ninetyDe();
                    Intake.IN();
                    ARM.DOWN();
                    lift.moveSlide(-450);
                    PIVOT.movePIVOT(-1400);
                    setPathState(31);
                }
                break;
            }
            case 31: {
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    PIVOT.movePIVOT(-400);
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        setPathState(4);
                    }
                }
                break;
            }

            case 4: {
                if (!PIVOT.isBusyPIVOT() && !lift.isBusyLift()) {
                    lift.moveSlide(-1500);
                    PIVOT.movePIVOT(-3700);
                    ARM.zeroFlip();
                    ARM.fwd();
                    setPathState(5);
                }
                break;

            }
            case 5: {
                    follower.followPath(score2);
                    setPathState(6);

                break;
            }
            case 6: {

                    if (!follower.isBusy()) {
                        Intake.STOP();
                        setPathState(7);
                    }
                break;
            }
            case 7: {
                if (!follower.isBusy()) {
                    ARM.UP();
                    setPathState(999);
                }
                break;
            }
            case 999: {//Park and move slides down
                if(pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(parkWill);
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.75) {
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
        telemetry.update();
    }
}
