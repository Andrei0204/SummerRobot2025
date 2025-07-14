package org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE;

@Config
@Autonomous(name = "sample", group = "11Saiba")
public class sample extends OpMode {
    private SaibaModules modules = null;
    Follower follower = null;
    private ElapsedTime pathtimer, actionTimer;
    GoBildaPinpointDriver odo;

    public static double x = 58, y = -10,
            lbrat = 15;//cm,
    public static double unghi = 0, ext = 0;//cm,
    public static Point punct1 = new Point(58, -11);
    public static Point punct2 = new Point(50, 10);
    public static Point punct3 = new Point(50, 13);
    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private STATES pathState = null;

    double cycles = 0;
    public static double i = 1.6;

    public void buildPaths() {
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case GOTOPRELOAD:
                //out
                follower.setTarget(new Pose(-54, 31, 1.3));
                modules.LIFT.setState(LIFT.State.FULL);
                modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                modules.INTUR.setState(INTUR.State.FRONT);
                if (modules.LIFT.getRealCurrentPosition() > 40)
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                //in
                if (follower.localization.getY() > 20) {

                    unghi = Math.toDegrees(Math.asin(punct1.getY() / lbrat));
                    ext = punct1.getX() - Math.sqrt(lbrat * lbrat - punct1.getY() * punct1.getY());
                    modules.EXT.setState(EXT.State.AUTO);
                    modules.EXT.setAuto(ext * EXT.TICKSCM);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(unghi);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

                }
                if (follower.reachedTargetAndStopped(4)
                        && follower.reachedHeading(Math.toRadians(8))
                        &&
                        modules.LIFT.getRealCurrentPosition() > 55 &&
                        modules.EXT.getRealCurrentPosition() > ext - 2) {
                    cycles++;
                    setPathState(STATES.SCOREANDCOLLECT);
                }
                break;
            case SCOREANDCOLLECT:
                if(pathtimer.milliseconds()>150){
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                }
                modules.INBRAT.setState(INBRAT.State.DOWN);
                if (pathtimer.milliseconds() > 500)
                    modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                if (pathtimer.milliseconds() > 700) {
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                    modules.LIFT.setState(LIFT.State.TRANSFER);
                }
                if (pathtimer.milliseconds() > 850) {
                    modules.INBRAT.setState(INBRAT.State.UP);

                    if (pathtimer.milliseconds() > 1000) {
                        modules.INTUR.setState(INTUR.State.TRANSFER);
                        modules.INROTATOR.setState(INROTATOR.State.THIRD);
                    }

                }
                if (pathtimer.milliseconds() > 1300) {
                    modules.EXT.setState(EXT.State.START);
                    setPathState(STATES.TRANSFER1);
                }
                break;
            case TRANSFER1:
                if (modules.EXT.getRealCurrentPosition() < 11 && modules.LIFT.getRealCurrentPosition() < 4) {
                    modules.OUTBRAT.setState(OUTBRAT.State.MID);
                    if (pathtimer.milliseconds() > 120)
                        modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    if (pathtimer.milliseconds() > 180)
                        modules.INCLAW.setState(INCLAW.State.RELEASE);
                    if (pathtimer.milliseconds() > 350) {

                        modules.LIFT.setState(LIFT.State.FULL);
                        if (cycles == 2) {
                            setPathState(STATES.SCORESECOND);
                        }
//                        else if(cycles==3)
//                        {
//                            setPathState(STATES.SCORETRD);
//                        }
                        else
                            setPathState(STATES.SCOREFST);
                    }
                } else
                    pathtimer.reset();
                break;
            case SCOREFST:
                modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                modules.INTUR.setState(INTUR.State.FRONT);
                if (modules.LIFT.getRealCurrentPosition() > 40)
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                //in
                if (pathtimer.milliseconds() > 200) {
                    unghi = Math.toDegrees(Math.asin(punct2.getY() / lbrat));
                    ext = punct2.getX() - Math.sqrt(lbrat * lbrat - punct2.getY() * punct2.getY());
                    modules.EXT.setState(EXT.State.AUTO);
                    modules.EXT.setAuto(ext * EXT.TICKSCM);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(unghi);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                    if (
                            modules.LIFT.getRealCurrentPosition() > 56 &&
                                    modules.EXT.getRealCurrentPosition() > ext - 1) {
                        setPathState(STATES.SCOREANDCOLLECT);
                        cycles++;
                    }
                }
                break;
            case SCORESECOND:
                unghi = Math.toDegrees(Math.asin(punct3.getY() / lbrat));
                ext = punct3.getX() - Math.sqrt(lbrat * lbrat - punct3.getY() * punct3.getY());
                modules.EXT.setState(EXT.State.AUTO);
                modules.EXT.setAuto(ext * EXT.TICKSCM);
                modules.INTUR.setState(INTUR.State.AUTOSPECI);
                modules.INTUR.setangle(unghi);
                modules.INROTATOR.setState(INROTATOR.State.FOURTH);
//                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);

                if (modules.LIFT.getRealCurrentPosition() > 40)
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                if (modules.LIFT.getRealCurrentPosition() > 54) {
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    if (pathtimer.milliseconds() > 100)
                        modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    if (pathtimer.milliseconds() > 200) {
                        modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        modules.LIFT.setState(LIFT.State.TRANSFER);
                        setPathState(STATES.SCORETRD);
                    }
                } else pathtimer.reset();
                break;
            case SCORETRD:
                if (modules.LIFT.getRealCurrentPosition() < 10) {
                    follower.setTarget(new Pose(-60, 31,  Math.toRadians(90)));
                    if (follower.reachedTargetAndStopped(2.5) && follower.reachedHeading(Math.toRadians(4)) && modules.EXT.getRealCurrentPosition() > ext - 1.5) {
                        setPathState(STATES.TRANSFER3);
                    }
                }
                break;
            case TRANSFER3:
                modules.INBRAT.setState(INBRAT.State.DOWN);

                if (pathtimer.milliseconds() > 200) {
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                }
                if (pathtimer.milliseconds() > 400) {
                    modules.INBRAT.setState(INBRAT.State.UP);

                    if (pathtimer.milliseconds() > 600) {
                        modules.INTUR.setState(INTUR.State.TRANSFER);
                        modules.INROTATOR.setState(INROTATOR.State.THIRD);
                    }
                }
                if (pathtimer.milliseconds() > 900) {
                    modules.EXT.setState(EXT.State.START);
                    setPathState(STATES.TRANSFERTRD);
                }
                break;
            case TRANSFERTRD:
                follower.setTarget(new Pose(-60, 28, Math.toRadians(90)));
                if (modules.EXT.getRealCurrentPosition() < 9.5 && modules.LIFT.getRealCurrentPosition() < 4) {
                    modules.OUTBRAT.setState(OUTBRAT.State.MID);
                    if (pathtimer.milliseconds() > 120)
                        modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    if (pathtimer.milliseconds() > 220)
                        modules.INCLAW.setState(INCLAW.State.RELEASE);
                    if (pathtimer.milliseconds() > 350) {
                        modules.LIFT.setState(LIFT.State.FULL);
                        setPathState(STATES.DEPOSITTRD);
                    }
                }
                else pathtimer.reset();
                break;
            case DEPOSITTRD:

                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);

                if (modules.LIFT.getRealCurrentPosition() > 40) {
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);

                }
                if(modules.LIFT.getRealCurrentPosition() > 52)
                {
                    if(pathtimer.milliseconds()>800) {
                        modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                        modules.INTUR.setState(INTUR.State.FRONT);
                        modules.EXT.setState(EXT.State.ZERO);
                    }
                    if(pathtimer.milliseconds()>1150)
                        setPathState(STATES.GOTOSUB);
                }
                else
                    pathtimer.reset();
                break;
            case GOTOSUB:
                follower.setTarget(new Pose(23,130,0));
                if(follower.localization.getY()>60)
                {

                    modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                    modules.LIFT.setState(LIFT.State.TRANSFER);
                }

                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(STATES pState) {
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
        telemetry.addData("x", follower.odo.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("y", follower.odo.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("h", follower.odo.getPosition().getHeading(AngleUnit.RADIANS));
        telemetry.addData("target", follower.reachedTarget(2));
        telemetry.addData("targetandstop", follower.reachedTargetAndStopped(2));
        telemetry.addData("heading", follower.reachedHeading(Math.toRadians(4)));
        telemetry.addData("cycles", cycles);
        telemetry.addData("states", pathState);

        telemetry.update();

    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
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
        modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        modules.update();
        modules.OUTBRAT.setState(OUTBRAT.State.MID);
        follower.update();

        telemetry.update();

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {

        setPathState(STATES.GOTOPRELOAD);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    public enum STATES {
        GOTOPRELOAD,
        SCOREANDCOLLECT,
        TRANSFER1,
        SCOREFST,
        SCORESECOND,
        TRANSFER3,
        SCORETRD,
        TRANSFERTRD,
        DEPOSITTRD,
        GOTOSUB,
        STOP
    }

}