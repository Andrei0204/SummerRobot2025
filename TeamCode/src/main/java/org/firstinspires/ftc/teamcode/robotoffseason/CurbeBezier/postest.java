package org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier.Follower;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "custom", group = "11Saiba")
public class postest extends OpMode {
    private SaibaModules modules = null;
    Follower follower = null;
    private ElapsedTime pathtimer, actionTimer;
    GoBildaPinpointDriver odo;
    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;


    public static double i = 1.6;

    public void buildPaths() {
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.setTarget(new Pose(84, 9, 0));
                modules.LIFT.setState(LIFT.State.SCORESPECI);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                if (modules.LIFT.getRealCurrentPosition() > 10)
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                if (follower.localization.getX() > 83) {
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);

                    follower.setTarget(new Pose(48, -69, -0.86));
                    setPathState(2);
                }
                break;
            case 2: //primu
                if (follower.localization.getX() < 60) {
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    modules.LIFT.setState(LIFT.State.START);
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(45);
                    modules.EXT.setState(EXT.State.AUTO);
                    modules.EXT.setAuto(34 * EXT.TICKSCM);
                    modules.INROTATOR.setState(INROTATOR.State.THIRD);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                }
                if (follower.reachedTargetAndStopped(3)) {

                    setPathState(3);
                }
                break;
            case 3:
                if (pathtimer.milliseconds() > 100)
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                if (pathtimer.milliseconds() > 300)
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                if (pathtimer.milliseconds() > 400) {
                    follower.setTarget(new Pose(44, -80, -2));
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);
                    modules.INTUR.setState(INTUR.State.DIAG);
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.reachedHeading(Math.toRadians(10))) {
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathtimer.milliseconds() > 100) {
                    follower.setTarget(new Pose(45.5, -94, -0.83));
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(45);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.reachedHeading(Math.toRadians(3)) && follower.reachedTargetAndStopped(3.5)) {
                    setPathState(-6);
                } else
                    pathtimer.reset();
                break;
            case -6:

                if (pathtimer.milliseconds() > 100)
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                if (pathtimer.milliseconds() > 300)
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                if (pathtimer.milliseconds() > 500) {
                    follower.setTarget(new Pose(44, -96, -2));
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);
                    modules.INTUR.setState(INTUR.State.DIAG);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.reachedHeading(Math.toRadians(10))) {
                    modules.EXT.setAuto(32 * EXT.TICKSCM);
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathtimer.milliseconds() > 100) {
                    follower.setTarget(new Pose(42, -115, -0.73));
                    modules.INTUR.setState(INTUR.State.AUTOSPECI);
                    modules.INTUR.setangle(0);
                    modules.INROTATOR.setState(INROTATOR.State.SECOND);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.reachedHeading(Math.toRadians(5)) && follower.reachedTargetAndStopped(4)) {
                    modules.EXT.setAuto(34 * EXT.TICKSCM);
                    setPathState(-9);
                } else
                    pathtimer.reset();
                break;
            case -9:

                if (modules.INBRAT.getState() == INBRAT.State.UP)
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                if (pathtimer.milliseconds() > 150)
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                if (pathtimer.milliseconds() > 300)
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                if (pathtimer.milliseconds() > 450) {
                    modules.EXT.setState(EXT.State.ZERO);
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);
                    modules.INTUR.setState(INTUR.State.THROW);
                    modules.INROTATOR.setState(INROTATOR.State.FOURTH);
                    follower.setTarget(new Pose(8, -100, 0));
                    setPathState(10);
                }
                break;
            case 10:
                follower.lateralmultiplier=1.3;
                if(modules.LIFT.getState()== LIFT.State.SCORESPECI && follower.localization.getX()<70)
                    modules.LIFT.setState(LIFT.State.START);
                if (follower.reachedTarget(4)) {
                    modules.INTUR.setState(INTUR.State.DIAG);
                    if (pathtimer.milliseconds() > 70) {
                        modules.INCLAW.setState(INCLAW.State.RELEASE);
                        modules.OUTCLAW.setState(OUTCLAW.State.SECURE);

                    }
                    if(pathtimer.milliseconds()>160)
                        setPathState(11);
                } else
                    pathtimer.reset();
                break;
            case 11:
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                modules.LIFT.setState(LIFT.State.SCORESPECI);
                follower.lateralmultiplier=i;
                follower.setTarget(new Pose(91.5, 1, Math.toRadians(5)));
                if(follower.localization.getY()>-50) {
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                }
                if(follower.reachedTarget(4.5))
                {
                    modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                    follower.setTarget(new Pose(8, -85, 0));
                    setPathState(10);
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
        telemetry.addData("x", follower.odo.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("y", follower.odo.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("h", follower.odo.getPosition().getHeading(AngleUnit.RADIANS));
        telemetry.addData("target", follower.reachedTarget(2));
        telemetry.addData("targetandstop", follower.reachedTargetAndStopped(2));
        telemetry.addData("heading", follower.reachedHeading(Math.toRadians(4)));
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
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {

        setPathState(1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}