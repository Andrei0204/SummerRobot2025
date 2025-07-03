package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
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
import org.opencv.core.Mat;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "kinematicstest", group = "11Saiba")
public class kinematicstest extends OpMode {
    private SaibaModules modules = null;
    private Follower follower;
    private Timer opmodeTimer;
    private ElapsedTime pathtimer, actionTimer;

    /**
                 * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    private final Pose startPose = new Pose(0, 0, 0);
    private Path scorePreload;

    public static double x=0,y=0 ,lbrat=15;//cm,
    public static double unghi=0,ext=0;//cm,
    public static Point punct=new Point(x,y);

    public void buildPaths() {
    }
    public boolean condEX(){
        if(Math.sqrt(punct.getX()*punct.getX() +punct.getY()*punct.getY())>= lbrat &&
        Math.abs(punct.getY())<=lbrat*Math.cos(Math.toRadians(26.1))&&
                punct.getX()>0)
            return true;
        return false;

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                modules.INTUR.setState(INTUR.State.FRONT);
                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                punct=new Point(x,y);
                unghi=Math.toDegrees(Math.asin(punct.getY()/lbrat));
                ext= punct.getX()-Math.sqrt(lbrat*lbrat -punct.getY()*punct.getY());
                if(gamepad1.a && condEX())
                {
                    modules.EXT.setState(EXT.State.AUTO);
                    modules.EXT.setAuto(ext*EXT.TICKSCM);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(unghi);

                    setPathState(-2);
                }
                break;
            case -2:
                if(gamepad1.b)
                {
                    setPathState(-1);
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
        modules.update();
        telemetry.addData("punct",punct);
        telemetry.addData("unghi",unghi);
        telemetry.addData("ext",ext);
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {

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
        setPathState(-1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}