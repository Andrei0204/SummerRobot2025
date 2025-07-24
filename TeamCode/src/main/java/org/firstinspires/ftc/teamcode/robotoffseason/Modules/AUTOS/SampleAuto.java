package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import android.media.midi.MidiOutputPort;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

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
@Disabled

@Config
@Autonomous(name = "SampleAuto", group = "11Saiba")
public class   SampleAuto extends OpMode {
    private SaibaModules modules = null;
    private Follower follower;
    private Timer opmodeTimer;
    private ElapsedTime pathtimer, actionTimer;
    public static double i = 14;
    public static double i2 = 0;
    public static double d = 132;
    public static double grade = -4;
    /**
     * This is the variable where we store the state of our auto.
     * <p>
     * It is used by the pathUpdate method.
     */
    private int pathState;


    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(7, 112, Math.toRadians(-90));


    /**
     * Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle.
     */
    private final Pose scorePose = new Pose(14, 133, Math.toRadians(-27));


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


        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
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


                modules.LIFT.setState(LIFT.State.AUTO);
                modules.LIFT.setAuto(68*LIFT.TICKSCM);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                modules.INTUR.setState(INTUR.State.FRONT);

                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:

                if (modules.LIFT.getRealCurrentPosition() > 55 && !follower.isBusy()) {

                    if (i2 == 0) {
                        follower.holdPoint(new Point(scorePose), Math.toRadians(-27));
                        actionTimer.reset();
                        i2++;
                    }
                }
                if (modules.LIFT.getRealCurrentPosition() > 55) {
                    modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);

                }
                if (i2 != 0 && actionTimer.milliseconds() > 300) {
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    setPathState(-1);
                    //                    modules.EXT.setState(EXT.State.AUTO);
//                    modules.EXT.setAuto(42 * EXT.TICKSCM);
//                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
//                    modules.INTUR.setangle(0);
//                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
//                    setPathState(2)
                    i2 = 0;
                }
                break;
            case 2:

                if (follower.getAcceleration().getMagnitude() > 1 && modules.EXT.getRealCurrentPosition()>36)
                    actionTimer.reset();
                else {

                    if (actionTimer.milliseconds() > 0) {
                        modules.INBRAT.setState(INBRAT.State.DOWN);
                    }
                    if (actionTimer.milliseconds() > 70) {
                        modules.INCLAW.setState(INCLAW.State.INTERIOR);
                        modules.LIFT.setState(LIFT.State.TRANSFER);
                        modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                        modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (pathtimer.milliseconds() > 100) {
                    modules.INBRAT.setState(INBRAT.State.UP);
                    modules.INTUR.setState(INTUR.State.TRANSFER);
                    modules.EXT.setState(EXT.State.START);
                    modules.INROTATOR.setState(INROTATOR.State.THIRD);
                }
                if (modules.EXT.getRealCurrentPosition() < 8)
                    setPathState(4);
                break;
            case 4:
                modules.OUTBRAT.setState(OUTBRAT.State.MID);
                if (pathtimer.milliseconds() > 150) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                }
                if (pathtimer.milliseconds() > 230) {
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    follower.holdPoint(new Point(scorePose), Math.toRadians(grade));
                    modules.LIFT.setState(LIFT.State.FULL);
                    modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    modules.EXT.setState(EXT.State.AUTO);
                    modules.EXT.setAuto(30 * EXT.TICKSCM);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathtimer.milliseconds()>100)
                {
                    modules.INTUR.setState(INTUR.State.FRONT);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                }
                if(modules.OUTCLAW.getState()!= OUTCLAW.State.RELEASE)
                    actionTimer.reset();
                if(modules.LIFT.getRealCurrentPosition()>24)
                {
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                }
                if(actionTimer.milliseconds()>200)
                {
                    modules.INBRAT.setState(INBRAT.State.UP);
                    modules.INTUR.setState(INTUR.State.TRANSFER);
                    modules.EXT.setState(EXT.State.START);
                    modules.INROTATOR.setState(INROTATOR.State.THIRD);
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

        telemetry.update();
        modules.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        i2 = 0;
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