package org.firstinspires.ftc.teamcode.robotoffseason.Misu;

import android.media.midi.MidiDeviceInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier.Follower;
import org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier.GoBildaPinpointDriver;
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

import java.sql.Time;


@Autonomous
public class SampleMisu extends LinearOpMode {

    private SaibaModules Modules;
    Follower Drive;
    GoBildaPinpointDriver Odometry;
    Limelight camera = null;

    ElapsedTime timer,globalTimer;

    int cycles = 0;

    public enum State
    {
        START,
        GO_TO_PRELOAD_SETUP_FOR_1ST,
        RELEASE_AND_COLLECT_1ST,
        TRANSFER,
        UP_AND_SETUP_FOR_2ND,
        RELEASE_AND_COLLECT_2ND,
        UP_AND_SETUP_FOR_3RD,
        RELEASE_AND_COLLECT_3RD,
        UP_3RD,
        RELEASE_3RD,
        GO_TO_SUB_PREP,
        GO_TO_SUB,
        DETECT,
        COLLECT,
        TRANSFER_AND_GO_TO_BASKET,
        AUX,
        GO_TO_PARK,
        WINDOWS_OK_ERROR,
        CU_VACA

    }

    State currentState = State.START, nextState = null;

    State afterOhShit = null;

    int ok = 0;
    int ecycles = 0;
    double x, y;
    Pose punct;
    double unghi, ext, lbrat = 15;

    @Override
    public void runOpMode() throws InterruptedException{

        // Initializations (Constructors)

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Odometry.setOffsets(1.9, -4.6, DistanceUnit.CM);
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        Odometry.resetPosAndIMU();

        timer = new ElapsedTime();
        globalTimer = new ElapsedTime();

        camera = new Limelight(hardwareMap);
        camera.pipelineZero();

        Drive = new Follower(hardwareMap, Odometry);
        Modules = new SaibaModules(hardwareMap, 1);
        Modules.EXT.EXT.resetEncoder();
        Modules.LIFT.LIFT1.resetEncoder();
        Modules.LIFT.LIFT2.resetEncoder();
        Modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
        Modules.update();




        //waitForStart(); or while(!isStopRequested() && opModeInInit())

        while(!isStopRequested() && opModeInInit())
        {
            Modules.OUTBRAT.setState(OUTBRAT.State.MID);
            Modules.update();
        }



        while(opModeIsActive() && !isStopRequested())
        {
            switch(currentState)
            {
                case START:
                    setNextState(State.GO_TO_PRELOAD_SETUP_FOR_1ST);
                    break;
                case GO_TO_PRELOAD_SETUP_FOR_1ST:

                    Drive.setTarget(PRELOAD_POSE);
                    Modules.LIFT.setState(LIFT.State.FULL);
                    Modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                    Modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                    Modules.INTUR.setangle(INTUR.FRONT);

                    if(Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.samplepos) <= 0.1){
                        Modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    }

                    if(Drive.localization.getY()>12)
                    {
                        Modules.setIntakeKinematic(FIRST_SAMPLE);
                    }

                    if(Drive.reachedTargetAndStopped(4) &&
                        Drive.reachedHeading(Math.toRadians(5)) &&
                        Modules.LIFT.getRealCurrentPosition() > 55 &&
                        Modules.EXT.atTarget())
                    {
                        setNextState(State.RELEASE_AND_COLLECT_1ST);
                    }



                    break;
                case RELEASE_AND_COLLECT_1ST:
                    Modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    Modules.INBRAT.setState(INBRAT.State.DOWN);
                    if(timer.milliseconds()>200)
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    if(timer.milliseconds()>550)
                    {
                        Modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        Modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        Modules.INCLAW.setState(INCLAW.State.SECURE);
                        Modules.LIFT.setState(LIFT.State.TRANSFER);
                        if(timer.milliseconds()>750)
                        {

                            Modules.INBRAT.setState(INBRAT.State.UP);
                            if (timer.milliseconds() > 850) {
                                Modules.EXT.setState(EXT.State.MID);//aici era start
                                Modules.INTUR.setState(INTUR.State.TRANSFER);
                                Modules.INROTATOR.setState(INROTATOR.State.THIRD);
                                if(timer.milliseconds()>1150) {

                                    Modules.OUTBRAT.setState(OUTBRAT.State.MID);
                                    if(Modules.LIFT.getRealCurrentPosition() < 4  && Modules.EXT.atTarget()) // && EXT.atTarget()
                                    {
                                        Modules.EXT.setState(EXT.State.START);
                                        if(Modules.EXT.atTarget())
                                            setNextState(State.TRANSFER);
                                    }

                                }
                            }
                        }
                    }
                    break;
                case TRANSFER:
                    if (timer.milliseconds() > 120)
                        Modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    if (timer.milliseconds() > 180) {
                        Modules.INCLAW.setState(INCLAW.State.RELEASE);
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    }
                    if(timer.milliseconds()>300)
                    {
                        Modules.INTUR.setState(INTUR.State.FRONT);
                        cycles++;
                        if(cycles == 1)
                            setNextState(State.UP_AND_SETUP_FOR_2ND);
                        if(cycles == 2)
                            setNextState(State.UP_AND_SETUP_FOR_3RD);
                        if(cycles == 3)
                            setNextState(State.UP_3RD);//cycle if
                    }

                    break;
                case UP_AND_SETUP_FOR_2ND:
                    Modules.LIFT.setState(LIFT.State.FULL);
                    Modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                    Modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                    Modules.INTUR.setangle(INTUR.FRONT);

                    if(Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.samplepos) <= 0.1){
                        Modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    }

                    Modules.setIntakeKinematic(SECOND_SAMPLE);
                    Modules.INROTATOR.setState(INROTATOR.State.FOURTH);

                    if(Modules.LIFT.getRealCurrentPosition() > 54 && Modules.EXT.getRealCurrentPosition() > Modules.getKinematicExtension() - 2)
                    {
                        setNextState(State.RELEASE_AND_COLLECT_2ND);
                    }

                    break;
                case RELEASE_AND_COLLECT_2ND:
                    Modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    Modules.INBRAT.setState(INBRAT.State.DOWN);
                    if(timer.milliseconds()>200)
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    if(timer.milliseconds()>550)
                    {
                        Modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        Modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        Modules.INCLAW.setState(INCLAW.State.SECURE);
                        Modules.LIFT.setState(LIFT.State.TRANSFER);
                        if(timer.milliseconds()>750)
                        {

                            Modules.INBRAT.setState(INBRAT.State.UP);
                            if (timer.milliseconds() > 850) {
                                Modules.EXT.setState(EXT.State.MID);
                                Modules.INTUR.setState(INTUR.State.TRANSFER);
                                Modules.INROTATOR.setState(INROTATOR.State.THIRD);
                                if(timer.milliseconds()>1150) {

                                    Modules.OUTBRAT.setState(OUTBRAT.State.MID);
                                    if(Modules.LIFT.getRealCurrentPosition()<4 && Modules.EXT.atTarget())
                                    {
                                        Modules.EXT.setState(EXT.State.START);
                                        if(Modules.EXT.atTarget())
                                            setNextState(State.TRANSFER);
                                    }

                                }
                            }
                        }
                    }
                    break;
                case UP_AND_SETUP_FOR_3RD:
                    Modules.LIFT.setState(LIFT.State.FULL);
                    Modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                    Modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                    Modules.INTUR.setangle(INTUR.FRONT);

                    Drive.setTarget(THIRD_POSE);

                    if(Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.samplepos) <= 0.1){
                        Modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    }

                    Modules.setIntakeKinematic(THIRD_SAMPLE);
                    Modules.INROTATOR.setState(INROTATOR.State.FOURTH);

                    if(Modules.LIFT.getRealCurrentPosition()>55 && Modules.EXT.atTarget() && Drive.reachedTargetAndStopped(4))
                    {
                        setNextState(State.RELEASE_AND_COLLECT_3RD);
                    }
                    break;
                case RELEASE_AND_COLLECT_3RD:
                    Modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                    Modules.INBRAT.setState(INBRAT.State.DOWN);
                    if(timer.milliseconds()>200)
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    if(timer.milliseconds()>550)
                    {
                        Modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        Modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        Modules.INCLAW.setState(INCLAW.State.SECURE);
                        Modules.LIFT.setState(LIFT.State.TRANSFER);
                        if(timer.milliseconds()>750)
                        {

                            Modules.INBRAT.setState(INBRAT.State.UP);
                            if (timer.milliseconds() > 850) {
                                Modules.EXT.setState(EXT.State.MID);
                                Modules.INTUR.setState(INTUR.State.TRANSFER);
                                Modules.INROTATOR.setState(INROTATOR.State.THIRD);
                                if(timer.milliseconds()>1150) {

                                    Modules.OUTBRAT.setState(OUTBRAT.State.MID);
                                    if(Modules.LIFT.getRealCurrentPosition() < 4 && Modules.EXT.atTarget())
                                    {
                                        Modules.EXT.setState(EXT.State.START);
                                        if(Modules.EXT.atTarget())
                                            setNextState(State.TRANSFER);
                                    }

                                }
                            }
                        }
                    }
                    break;
                case UP_3RD:
                    Modules.LIFT.setState(LIFT.State.FULL);
                    Modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                    Modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
                    Modules.INTUR.setangle(INTUR.FRONT);

                    if(Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.samplepos) <= 0.1){
                        Modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    }

                    Modules.EXT.setState(EXT.State.ZERO);
                    Modules.INTUR.setState(INTUR.State.AUTOSPECI);

                    if(Modules.LIFT.getRealCurrentPosition()>55 && Drive.reachedTargetAndStopped(6.6))
                    {
                        setNextState(State.RELEASE_3RD);
                    }
                    break;
                case RELEASE_3RD:
                    Modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);

                    if(timer.milliseconds()>200)
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    if(timer.milliseconds()>550)
                    {
                        Modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        Modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        if(timer.milliseconds()>750) //  && Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.transferpos) <= 0.1
                        {
                            setNextState(State.GO_TO_SUB_PREP);
                        }
                    }
                    break;
                case GO_TO_SUB_PREP:
                    Drive.setTarget(SUB_PREP);
                    Modules.LIFT.setState(LIFT.State.TRANSFER);
                    if(Drive.reachedTarget(20))
                    {
                        setNextState(State.GO_TO_SUB);
                    }
                    break;
                case GO_TO_SUB:
                    Drive.setTarget(SUB_POSE);
                    if(Drive.reachedTargetAndStopped(4))
                    {
                        setNextState(State.DETECT);
                    }
                    break;
                case DETECT:
                    Pose2d sex=camera.getdistanceVector(0,"red",0.8);
                    if (sex.getY() != 0 && sex.getX() != 0) {

                        Modules.INTUR.setState(INTUR.State.FRONT);
                        Modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

                        x = sex.getX()-2;
                        y = sex.getY() + 13;
                        punct = new Pose(x, y);
                        unghi = Math.toDegrees(Math.asin(punct.getY() / lbrat));
                        if (camera.isLateral("red") == 1 || camera.isLateral("red") == 0) {
                            Modules.INROTATOR.setAngle(unghi);
                            if (camera.isLateral("red") == 0)
                                x = sex.getX();
                        } else {
                            Modules.INROTATOR.setAngle(unghi - 90);
                            x = sex.getX() ;

                        }
//                        if (x < 24)
//                            x = sex.getX() - 3;
                        ext = punct.getX() - Math.sqrt(lbrat * lbrat - punct.getY() * punct.getY());
                        Modules.EXT.setState(EXT.State.AUTO);
                        Modules.EXT.setAuto(ext * EXT.TICKSCM);
                        Modules.INTUR.setState(INTUR.State.AUTOSPECI);
                        Modules.INTUR.setangle(unghi);
                        if(Modules.EXT.atTarget())
                        {
                            setNextState(State.COLLECT);
                        }
                    }

                    break;
                case COLLECT:
                    if(timer.milliseconds()>250) {
                        Modules.INBRAT.setState(INBRAT.State.DOWN);
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);

                    }
                    if(timer.milliseconds()>550)
                    {
                        Modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                        Modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                        Modules.INCLAW.setState(INCLAW.State.SECURE);
                        Modules.LIFT.setState(LIFT.State.TRANSFER);
                        if(timer.milliseconds()>750)
                        {
                            Modules.INBRAT.setState(INBRAT.State.UP);
                            if (timer.milliseconds() > 850) {
                                Modules.EXT.setState(EXT.State.MID);
                                Modules.INTUR.setState(INTUR.State.TRANSFER);

                                Drive.setTarget(BASKET);
                                if(timer.milliseconds()>1150) {
                                    Modules.INROTATOR.setState(INROTATOR.State.THIRD);
                                    Modules.OUTBRAT.setState(OUTBRAT.State.MID);
                                    if(Modules.LIFT.getRealCurrentPosition() < 4 && Modules.EXT.atTarget())
                                    {
                                        Modules.EXT.setState(EXT.State.START);
                                        if(Modules.EXT.atTarget())
                                            setNextState(State.TRANSFER_AND_GO_TO_BASKET);
                                    }

                                }
                            }
                        }
                    }
                    break;
                case TRANSFER_AND_GO_TO_BASKET:

                    Drive.setKP(0.06);
                    Drive.setKD(0.012);
                    Drive.lateralmultiplier = 1.6;
                    if (timer.milliseconds() > 120)
                        Modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    if (timer.milliseconds() > 180) {
                        Modules.INCLAW.setState(INCLAW.State.RELEASE);
                        Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                    }
                    if(timer.milliseconds()>500)
                    {
                        Modules.INTUR.setState(INTUR.State.FRONT);
                        Modules.LIFT.setState(LIFT.State.FULL);
                        Modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                        if(Math.abs(Modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.samplepos) <= 0.1){
                            Modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                        }
                        if(Drive.reachedTargetAndStopped(4))
                        {
                            Modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);

                            if(Modules.LIFT.getRealCurrentPosition()>55)
                            {
                                ecycles++;
                                setNextState(State.AUX);
                                Drive.setKP(0.09);
                                Drive.setKD(0.015);
                                Drive.lateralmultiplier = 1;

                                if(ecycles == 4) {
                                    setNextState(State.GO_TO_PARK);//cycle if
                                }
                            }

                        }
                    }
                    break;
                case AUX:
                    if(timer.milliseconds()>0)
                    {
                        setNextState(State.RELEASE_3RD);
                    }
                    break;
                case GO_TO_PARK:
                    break;
                case WINDOWS_OK_ERROR:
                    Modules.LIFT.setState(LIFT.State.FULL);
                    Modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);

                    Drive.setTarget(OH_SHIT_POSE);
                    ok = 0;
                    if(Drive.reachedTarget(8))
                        setNextState(afterOhShit);
                    break;
                case CU_VACA:
                    mergiCuVaca();
                    break;
            }

//            if(Modules.getLiftCurrent() > 6.9 && ok == 0)
//            {
//                afterOhShit = currentState;
//                nextState = afterOhShit;
//                ok = 1;
//                setNextState(State.WINDOWS_OK_ERROR);
//            }

            Odometry.update();
            Drive.update();
            Modules.update();

            Telemetry();

            currentState = nextState;
        }



    }

    public void mergiCuVaca() {
        while(opModeIsActive());
    }

    public void Telemetry()
    {
        telemetry.addData("x", Drive.odo.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("y", Drive.odo.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("h", Drive.odo.getPosition().getHeading(AngleUnit.RADIANS));
        telemetry.addData("Current State", currentState.toString());
        telemetry.addData("AMPS", Modules.getLiftCurrent());
        telemetry.update();
    }

    public void setNextState(State newState)
    {
        this.nextState = newState;
        timer.reset();
    }

    public static Pose PRELOAD_POSE = new Pose(-54, 31, Math.toRadians(75));
    public static Pose THIRD_POSE = new Pose(-60, 31, Math.toRadians(90));
    public static Pose SUB_PREP = new Pose(0, 140, Math.toRadians(15));
    public static Pose SUB_POSE = new Pose(39, 140, Math.toRadians(0));

    public static Pose BASKET = new Pose(-47, 23.5, 1);

    public static Pose OH_SHIT_POSE = new Pose(-54, 50, Math.toRadians(75));

    public static Point FIRST_SAMPLE = new Point(58, -12);
    public static Point SECOND_SAMPLE = new Point(52, 9);
    public static Point THIRD_SAMPLE = new Point(50, 13);


}
