package org.firstinspires.ftc.teamcode.robotoffseason.Modules.AUTOS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled

@Autonomous(name = "Autoredsob", group = "11Saiba")
public class Specisaibavarsob extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private Telemetry telemetryA;
    private SaibaModules modules = null;
    private int pathState;
    private final Pose startPose = new Pose(8, 65, 0);

    private final Pose scorePose = new Pose(42, 69, 0);
    private final Pose S1 = new Pose(16, 25, 0);
    private final Pose S1rot = new Pose(16, 20, 0);
    private final Pose S2 = new Pose(14, 14, 0);


    private Path scorePreload, rotates1;
    private PathChain firstsample, secondsample, thirdsample;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        rotates1 = new Path(new BezierLine(new Point(S1), new Point(S1rot)));
        rotates1.setLinearHeadingInterpolation(0, 5.2);

        firstsample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(S1)))
                .setConstantHeadingInterpolation(0)
                .build();
        secondsample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(S1rot), new Point(S2)))
                .setConstantHeadingInterpolation(0)
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                if (!follower.isBusy()
                ) {
                    follower.followPath(firstsample, true);

                        setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getY() < 30) {
                    modules.EXT.setState(EXT.State.FULL);
                    modules.INTUR.setState(INTUR.State.FRONT);
                    modules.INROTATOR.setState(INROTATOR.State.THIRD);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

                    if (!follower.isBusy() && modules.EXT.getRealCurrentPosition() > 46) {
                        modules.INBRAT.setState(INBRAT.State.DOWN);
                        actionTimer.reset();
                         setPathState(3);
                    }
                }
                break;
            case 3:
                if (pathTimer.milliseconds() > 150) {
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                }
                if (pathTimer.milliseconds() > 400) {
//follower.followPath(rotates1, true);
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);
                    modules.INTUR.setState(INTUR.State.DIAG);
                    modules.EXT.setState(EXT.State.ZERO);
                    actionTimer.reset();
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy() && modules.EXT.getRealCurrentPosition() < 7) {
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.milliseconds() > 100) {
                    follower.followPath(secondsample, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getHeading() > 5.8) {
                    modules.EXT.setState(EXT.State.FULL);
                    modules.INTUR.setState(INTUR.State.FRONT);
                    modules.INROTATOR.setState(INROTATOR.State.THIRD);
                    modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
                }
                if (!follower.isBusy() && modules.EXT.getRealCurrentPosition() > 46) {
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                    actionTimer.reset();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.milliseconds() > 150) {
                    modules.INCLAW.setState(INCLAW.State.SECURE);
                }
                if (pathTimer.milliseconds() > 400) {
                  //  follower.followPath(rotates1, true);
                    modules.INBRAT.setState(INBRAT.State.MIDDLE);
                    modules.INTUR.setState(INTUR.State.DIAG);
                    modules.EXT.setState(EXT.State.ZERO);
                    actionTimer.reset();
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy() && modules.EXT.getRealCurrentPosition() < 7) {
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    setPathState(-1);
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        modules.update();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        follower.telemetryDebug(telemetryA);
    }

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        opmodeTimer.reset();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        modules = new SaibaModules(hardwareMap, 1);
        modules.EXT.EXT.resetEncoder();
        modules.LIFT.LIFT1.resetEncoder();
        modules.LIFT.LIFT2.resetEncoder();
    }

    @Override
    public void init_loop() {

        if (gamepad1.a) {
            modules.EXT.EXT.resetEncoder();
            modules.LIFT.LIFT1.resetEncoder();
            modules.LIFT.LIFT2.resetEncoder();
        }
        modules.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}