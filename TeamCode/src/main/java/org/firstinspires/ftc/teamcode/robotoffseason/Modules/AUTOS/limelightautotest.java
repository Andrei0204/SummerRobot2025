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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.time.Year;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "LLTEST", group = "11Saiba")
public class limelightautotest extends OpMode {
    private SaibaModules modules = null;
    private Follower follower;
    private Timer opmodeTimer;
    private ElapsedTime pathtimer, actionTimer;
    /**
                 * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    public static double angle = 30;
    Limelight limelight = null;
    private final Pose startPose = new Pose(0, 0, 0);
    private Path scorePreload;
    double y = 0;
    public static double  d = 12;
    public static double  h = -1;
    public static double  k = 1;
    private PathChain gotohuman2;
    private PathChain firstsample, secondsample, thirdsample, firstspeci, gotohuman, score;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    Pose2d LL = new Pose2d(0, 0, 0);

    public void buildPaths() {
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:

                if (y == 0 && follower.poseUpdater.getAcceleration().getMagnitude() <3 ) {
                    LL = new Pose2d(0, 0, 0);

                    y = limelight.getDEGREES("red",0).getY();
                    LL = limelight.getdistanceVector(0, "red");
                }
                if (y != 0) {
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(150);
                    modules.INROTATOR.setState(INROTATOR.State.FOURTH);
                    modules.EXT.setState(EXT.State.AUTO);
                    scorePreload = new Path(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX(),
                            follower.getPose().getY() +k*(LL.getY()+h)/2.54) ));
                    scorePreload.setConstantHeadingInterpolation(0);
                    if((k*(LL.getY()+h)/2.54)>-2)
                        d=d-2;
                    else
                        d=12;
                    modules.EXT.setAuto((LL.getX()+d - 15) * EXT.TICKSCM);

                    follower.followPath(scorePreload, true);
                    setPathState(-22);
                }

                break;
            case-22:
                if(modules.EXT.getTarget()/EXT.TICKSCM-3 <modules.EXT.getRealCurrentPosition()&&!follower.isBusy())
                {
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                    setPathState(-33);
                }
                break;
            case-33:
                if(pathtimer.milliseconds()>300)
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                if(pathtimer.milliseconds()>600 )
                    modules.INBRAT.setState(INBRAT.State.UP);
                break;
            case 0:
                LL = new Pose2d(0, 0, 0);
                if (gamepad1.a)
                    LL = limelight.getdistanceVector(0, "red");
                if (LL.getX() != 0 && LL.getY() != 0) {
                    scorePreload = new Path(new BezierLine(follower.getPose(), new Pose(follower.getPose().getX() + LL.getX(), follower.getPose().getY() + LL.getY())));
                    scorePreload.setConstantHeadingInterpolation(0);
                    follower.followPath(scorePreload, true);
                    setPathState(2);
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
        telemetry.addData("y", y);
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("sex", LL);
        telemetry.update();
        modules.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        d=12;
        modules = new SaibaModules(hardwareMap, 1);
        modules.EXT.EXT.resetEncoder();
        modules.LIFT.LIFT1.resetEncoder();
        modules.LIFT.LIFT2.resetEncoder();
        pathtimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        limelight = new Limelight(hardwareMap);
        limelight.pipeline();

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
        setPathState(-1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}