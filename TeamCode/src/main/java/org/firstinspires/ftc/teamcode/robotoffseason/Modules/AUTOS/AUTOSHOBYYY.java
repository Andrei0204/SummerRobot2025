package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.misc.Limelight;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AUTOSHOBYYY", group = "11Saiba")
public class AUTOSHOBYYY extends OpMode {
    private SaibaModules modules = null;
    private Follower follower;
    private Timer opmodeTimer;
    private ElapsedTime pathtimer, actionTimer;
    /**
     * This is the variable where we store the state of our auto.
     * <p>
     * It is used by the pathUpdate method.
     */
    private int pathState;

    double y = 0;
    public static double d = 12;
    public static double h = -1;
    public static double k = 1;
    Pose2d LL = new Pose2d(0, 0, 0);
    public static double cycles = 0;
    Limelight limelight = null;
    public static double m = 50;
    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(8, 65, 0);
    private Pose detectpose = new Pose(0, 0, 0);

    /**
     * Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle.
     */
    private final Pose scorePose = new Pose(38.8, 69, 0);


    private final Pose BS1C1 = new Pose(21, 42, 0);
    private final Pose BS1FIN = new Pose(45, 33, 0);


    private final Pose S1C1 = new Pose(60, 36, 0);
    private final Pose S1C2 = new Pose(71, 18, 0);
    //    private final Pose S1FIN = new Pose(20 , 23, 0);
    private final Pose S1FIN = new Pose(18, 23, 0);


    private final Pose BS2C1 = new Pose(70, 25, 0);
    private final Pose BS2FIN = new Pose(53, 14, 0);


    //    private final Pose S2FIN = new Pose(20, 14, 0);
    private final Pose S2FIN = new Pose(18, 14, 0);

    private final Pose BS3C1 = new Pose(72, 17, 0);
    private final Pose BS3FIN = new Pose(53, 8, 0);
    private final Pose S3FIN = new Pose(11.7, 8, 0);

    private final Pose SP1C1 = new Pose(34, 64, 0);
    private final Pose SP1FIN = new Pose(39.5, 66, 0);

    private final Pose COLLECT = new Pose(14, 33, 0);
    private final Pose COLLECTWALL = new Pose(13.3, 33.3, 0);

    private final Pose COLLECTC1 = new Pose(31, 49, 0);
    private final Pose COLLECTC2 = new Pose(31, 33, 0);
//    private final Pose COLLECTC1 = new Pose(37 , 47, 0);
//    private final Pose COLLECTC2 = new Pose(44 , 33, 0);


    /**
     * Lowest (First) Sample from the Spike Mark
     */
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
//
//    /** Middle (Second) Sample from the Spike Mark */
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
//
//    /** Highest (Third) Sample from the Spike Mark */
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
//
//    /** Park Pose for our robot, after we do all of the scoring. */
//    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
//
//    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
//     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload;
    private PathChain gotohuman2;
    private PathChain firstsample, secondsample, thirdsample, firstspeci, gotohuman, score, scorefst;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private Path collectsample;
    double x = 0;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(0)
                .build();


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        firstsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(BS1C1), new Point(BS1FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(BS1FIN), new Point(S1C1), new Point(S1C2), new Point(S1FIN)))
                .setConstantHeadingInterpolation(0)
                .build();


        secondsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S1FIN), new Point(BS2C1), new Point(BS2FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(BS2FIN), new Point(S2FIN), new Point(S2FIN)))
                .setConstantHeadingInterpolation(0)

                .build();
        thirdsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S2FIN), new Point(BS3C1), new Point(BS3FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(BS3FIN), new Point(S3FIN)))
                .setConstantHeadingInterpolation(0)

                .build();
        firstspeci = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S3FIN), new Point(SP1C1), new Point(SP1FIN)))

                .setConstantHeadingInterpolation(0)
                .build();
        gotohuman = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP1FIN), new Point(COLLECTC1), new Point(COLLECTC2), new Point(COLLECT)))
                .setConstantHeadingInterpolation(0)
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(COLLECTWALL), new Point(SP1FIN)))
                .setConstantHeadingInterpolation(0)
                .build();
        scorefst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(COLLECTWALL), new Point(scorePose)))
                .setConstantHeadingInterpolation(0)
                .build();
        gotohuman2 = follower.pathBuilder()
//            .addPath(new BezierLine(COLLECT,COLLECTWALL))
//                .setConstantHeadingInterpolation(0)
//                .build();
                .addPath(new BezierLine(COLLECT, COLLECTWALL))
//                .setLinearHeadingInterpolation(Math.toRadians(15),0)
                .setConstantHeadingInterpolation(0)

                .build();

    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                limelight.limelight.start();
                limelight.pipeline();
                cycles = 1;
                LL = new Pose2d(0, 0, 0);
                y = 0;


                follower.followPath(scorePreload, true);
                modules.LIFT.setState(LIFT.State.SCORESPECI);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                modules.INCLAW.setState(INCLAW.State.RELEASE);
                modules.INROTATOR.setState(INROTATOR.State.THIRD);

                setPathState(1);
                break;
            case 1:

                if (modules.LIFT.getRealCurrentPosition() > 5) {
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    modules.EXT.setState(EXT.State.ZERO);
                }
                if (!follower.isBusy()) {
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.LIFT.setState(LIFT.State.START);
                    LL = new Pose2d(0, 0, 0);

                    setPathState(-1);
                }
                break;
            case -1:

                if (LL.getY() == 0 && follower.poseUpdater.getAcceleration().getMagnitude() <2.5 && pathtimer.milliseconds()>150) {
                    LL = new Pose2d(0, 0, 0);
                    LL = limelight.getdistanceVector(0, "red");

                }
                if (LL.getY() != 0) {
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(150);
                    modules.INROTATOR.setState(INROTATOR.State.FOURTH);
                    modules.EXT.setState(EXT.State.AUTO);
                    collectsample = new Path(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(),
                            follower.getPose().getY() + k * (LL.getY() + h) / 2.54)));
                    collectsample.setConstantHeadingInterpolation(0);
                    if ((k * (LL.getY() + h) / 2.54) > -2)
                        d = d - 2;
                    else
                        d = 12;
                    modules.EXT.setAuto((LL.getX() + d - 15) * EXT.TICKSCM);

                    follower.followPath(collectsample, true);
                    setPathState(-22);
                    limelight.limelight.stop();
                }
                break;
            case -22:
                if (modules.EXT.getTarget() / EXT.TICKSCM - 5 < modules.EXT.getRealCurrentPosition() && !follower.isBusy()) {
                    if (actionTimer.milliseconds() > 200) {
                        modules.INBRAT.setState(INBRAT.State.DOWN);
                        setPathState(-33);
                    }
                } else {
                    actionTimer.reset();
                }
                break;
            case -33:
                if (pathtimer.milliseconds() > 100)
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                if (pathtimer.milliseconds() > 200) {
                    modules.INBRAT.setState(INBRAT.State.UP);
                    modules.EXT.setState(EXT.State.ZERO);
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);

                    modules.INROTATOR.setState(INROTATOR.State.SECOND);
                    follower.followPath(gotohuman, true);
                    setPathState(-3);
                }
                break;
            case -3:
                if (pathtimer.milliseconds() > 500)
                    modules.INTUR.setState(INTUR.State.THROW);
                if (!follower.isBusy()) {

                    follower.followPath(gotohuman2, true);
                    setPathState(-10);
                }
                break;
            case -10:

                if (!follower.isBusy()) {
                    if (modules.INTUR.getState() == INTUR.State.THROW) {
                        actionTimer.reset();
                        modules.INTUR.setState(INTUR.State.DIAG);
                    } else {
                        if (actionTimer.milliseconds() > 30) {
                            modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                            modules.INCLAW.setState(INCLAW.State.RELEASE);
                        }
                        if (actionTimer.milliseconds() > 250) {
                            if (cycles == 1) {
                                follower.followPath(scorefst);
                            } else
                                follower.followPath(score);
                            modules.INCLAW.setState(INCLAW.State.RELEASE);
                            modules.LIFT.setState(LIFT.State.SCORESPECI);
                            modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                            setPathState(-11);
                        }
                    }

                }
                break;
            case -11:
                if (follower.getPose().getY() > 35)
                    modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                if (follower.getPose().getY() > 59) {

                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);

                }
                if (!follower.isBusy()) {
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);

                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LIFT.setState(LIFT.State.START);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    if (cycles == 2) {

                        limelight.limelight.start();
                        setPathState(6);
                        cycles++;
                    } else {
                        LL = new Pose2d(0, 0, 0);
                        limelight.limelight.stop();
                        follower.followPath(firstsample);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() || follower.getPose().getX() < 20) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    modules.INBRAT.setState(INBRAT.State.UP);
                    follower.followPath(secondsample, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || (follower.getPose().getX() < 20 && follower.getPose().getY() < 20)) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(thirdsample, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    actionTimer.reset();
                    setPathState(-4);
                }
                break;


            case -4:
                if (actionTimer.milliseconds() > 100) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathtimer.milliseconds() > 100) {

                    modules.LIFT.setState(LIFT.State.SCORESPECI);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);

                    follower.followPath(firstspeci, true);
                    setPathState(6);

                }
                break;
            case 6: //CICLAT
                if (follower.getPose().getY() > 35)
                    modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                if (follower.getPose().getY() > 59) {

                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);

                }
                if (!follower.isBusy()) {
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);

                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LIFT.setState(LIFT.State.START);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    if (cycles == 1) {
                        LL = new Pose2d(0, 0, 0);
                        follower.holdPoint(new Point(follower.getPose()), 0);
                        limelight.limelight.start();
                        setPathState(-1);
                        cycles++;
                    } else {
                        follower.followPath(gotohuman, true);

                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(gotohuman2, true);
                    setPathState(-7);
                    actionTimer.reset();
                }
                break;
            case -7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                if (actionTimer.milliseconds() > 100) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathtimer.milliseconds() > 100 && !follower.isBusy()) {

                    modules.LIFT.setState(LIFT.State.SCORESPECI);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);

                    follower.followPath(score, true);
                    setPathState(6);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathtimer.reset();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("LL", LL);
        telemetry.addData("dp", detectpose);
        telemetry.update();
        modules.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        d = 13;
        modules = new SaibaModules(hardwareMap, 1);
        modules.EXT.EXT.resetEncoder();
        modules.LIFT.LIFT1.resetEncoder();
        modules.LIFT.LIFT2.resetEncoder();
        pathtimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        limelight = new Limelight(hardwareMap);
        limelight.pipeline();
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        modules.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}