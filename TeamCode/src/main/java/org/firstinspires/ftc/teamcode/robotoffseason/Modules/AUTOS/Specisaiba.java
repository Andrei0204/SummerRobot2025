package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Autored", group = "11Saiba")
public class Specisaiba extends OpMode {
    private SaibaModules modules = null;
    private Follower follower;
    private Timer  opmodeTimer;
    private ElapsedTime pathtimer,actionTimer;
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 65, 0);

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(42.5, 69, 0);


    private final Pose BS1C1 = new Pose(21, 42, 0);
    private final Pose BS1FIN = new Pose(45 , 33, 0);


    private final Pose S1C1 = new Pose(60, 35, 0);
    private final Pose S1C2 = new Pose(71 , 19, 0);
    private final Pose S1FIN = new Pose(13 , 23, 0);





    private final Pose BS2C1 = new Pose(70, 25, 0);
    private final Pose BS2FIN = new Pose(53, 14, 0);


    private final Pose S2FIN = new Pose(13, 14, 0);

    private final Pose BS3C1 = new Pose(72, 17, 0);
    private final Pose BS3FIN = new Pose(53,  8, 0);
    private final Pose S3FIN = new Pose(11.3, 8, 0);

    private final Pose SP1C1 = new Pose(34, 64, 0);
    private final Pose SP1FIN = new Pose(39 , 67, 0);

    private final Pose COLLECT = new Pose(10.5 , 35, 0);

    private final Pose COLLECTC1 = new Pose(31 , 49, 0);
    private final Pose COLLECTC2 = new Pose(31 , 33, 0);
//    private final Pose COLLECTC1 = new Pose(37 , 47, 0);
//    private final Pose COLLECTC2 = new Pose(44 , 33, 0);



    /** Lowest (First) Sample from the Spike Mark */
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
    private Path scorePreload;
    private PathChain firstsample,secondsample,thirdsample,firstspeci,gotohuman,score;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        firstsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(BS1C1),new Point(BS1FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(BS1FIN), new Point(S1C1),new Point(S1C2),new Point(S1FIN)))
                .setConstantHeadingInterpolation(0)
                .build();




        secondsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S1FIN), new Point(BS2C1),new Point(BS2FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(BS2FIN),new Point(S2FIN)))
                .setConstantHeadingInterpolation(0)

                .build();
        thirdsample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S2FIN), new Point(BS3C1),new Point(BS3FIN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(BS3FIN),new Point(S3FIN)))
                .setConstantHeadingInterpolation(0)

                .build();
        firstspeci = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(S3FIN), new Point(SP1C1),new Point(SP1FIN)))

                .setConstantHeadingInterpolation(0)
                .build();
        gotohuman=follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP1FIN),new Point(COLLECTC1),new Point(COLLECTC2),new Point(COLLECT)))
                .setConstantHeadingInterpolation(0)
                .build();
        score=follower.pathBuilder()
                .addPath(new BezierLine(new Point(COLLECT),new Point(SP1FIN)))
                .setConstantHeadingInterpolation(0)
                .build();





//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(scorePreload);
                modules.EXT.setState(EXT.State.MID);
                modules.LIFT.setState(LIFT.State.SCORESPECI);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);

                setPathState(1);
                break;
            case 1:

                if(modules.LIFT.getRealCurrentPosition()>5) {
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    modules.EXT.setState(EXT.State.ZERO);
                }
                if(!follower.isBusy()) {
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.LIFT.setState(LIFT.State.START);
                    follower.followPath(firstsample,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() || follower.getPose().getX()<20) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    modules.INBRAT.setState(INBRAT.State.UP);
                    follower.followPath(secondsample,true);
                    setPathState(3 );
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()|| (follower.getPose().getX()<20 && follower.getPose().getY()<20)) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(thirdsample,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy() ) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    actionTimer.reset();
                    setPathState(-4);
                }
                break;


            case -4:
                if(actionTimer.milliseconds()>100) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathtimer.milliseconds()>100) {

                        modules.LIFT.setState(LIFT.State.SCORESPECI);
                        modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);


                        follower.followPath(firstspeci, true);
                        setPathState(6);

                }
                break;
            case 6: //CICLAT
                if(follower.getPose().getY()>40)
                    modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                if (follower.getPose().getY() > 60) {

                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);

                }
                if(!follower.isBusy()) {
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);

                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LIFT.setState(LIFT.State.START);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    follower.followPath(gotohuman,true);

                    setPathState(7);

                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    setPathState(8);
                    actionTimer.reset();
                }
                break;
            case 8:
                if(actionTimer.milliseconds()>100) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathtimer.milliseconds()>100&&!follower.isBusy()) {

                    modules.LIFT.setState(LIFT.State.SCORESPECI);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);

                    follower.followPath(score, true);
                    setPathState(6);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathtimer.reset();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        modules = new SaibaModules(hardwareMap, 1);
        modules.EXT.EXT.resetEncoder();
        modules.LIFT.LIFT1.resetEncoder();
        modules.LIFT.LIFT2.resetEncoder();
        pathtimer = new ElapsedTime();
        actionTimer=new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        modules.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}