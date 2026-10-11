package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier.Follower;
import org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.misc.Limelight;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled

@Config
@TeleOp(name = "kinematicstestfollower", group = "11Saiba")
public class kinematicstestfollower extends OpMode {
    private SaibaModules modules = null;
    private ElapsedTime pathtimer, actionTimer;

    /**
                 * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    private final Pose startPose = new Pose(0, 0, 0);
    private Path scorePreload;
    GoBildaPinpointDriver odo;
    public static double x=0,y=0 ,lbrat=15;//cm,
    public static double unghi=0,ext=0;//cm,
    public static Pose punct=new Pose(x,y);
    Follower follower = null;
    Limelight ll=null;
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
                follower.setTarget(new org.firstinspires.ftc.teamcode.roboDrive.math.Pose(84, 13, 0));

                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);
                if(follower.reachedTargetAndStopped(2.5) )
                {
                    Pose2d piece=ll.getdistanceVector(0,"red");

                    if(piece.getY()!=0 &&piece.getX()!=0) {
                        modules.INTUR.setState(INTUR.State.FRONT);
                        modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

                        x = piece.getX() - 2;
                        y = piece.getY() + 13;

                        if(ll.isLateral("red") ==1)
                        {
                            modules.INROTATOR.setAngle(unghi);
                        }
                        else {

                            modules.INROTATOR.setAngle(unghi - 90);
                            x = piece.getX() + 3;
                        }
                        if(x<24)
                            x = piece.getX() - 3;

                        punct = new Pose(x, y);
                        unghi = Math.toDegrees(Math.asin(punct.getY() / lbrat));
                        ext = punct.getX() - Math.sqrt(lbrat * lbrat - punct.getY() * punct.getY());
                        modules.EXT.setState(EXT.State.AUTO);
                        modules.EXT.setAuto(ext * EXT.TICKSCM);
                        modules.INTUR.setState(INTUR.State.AUTOSPECI);
                        modules.INTUR.setangle(unghi);
                        setPathState(-2);
                    }
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

        autonomousPathUpdate();
        modules.update();
        follower.update();
        telemetry.addData("punct",punct);
        telemetry.addData("unghi",unghi);
        telemetry.addData("ext",ext);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        ll =new Limelight(hardwareMap);
        ll.pipeline();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(1.9, -4.6, DistanceUnit.CM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        try {
            sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower = new Follower(hardwareMap, odo);
        modules = new SaibaModules(hardwareMap, 1);
        modules.EXT.EXT.resetEncoder();
        modules.LIFT.LIFT1.resetEncoder();
        modules.LIFT.LIFT2.resetEncoder();
        pathtimer = new ElapsedTime();
        actionTimer = new ElapsedTime();

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

        setPathState(-1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}