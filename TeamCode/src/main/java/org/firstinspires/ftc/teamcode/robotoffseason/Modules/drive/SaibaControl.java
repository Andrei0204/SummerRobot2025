package org.firstinspires.ftc.teamcode.robotoffseason.Modules.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.WEELIE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.PTO;

@Config
public class SaibaControl implements IRobotModule {
    public static double amp = 4;
    public static double sex = 0.45;
    public final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;
    public SaibaModules modules;
    ElapsedTime outtimer;
    ElapsedTime intaketimer;
    ElapsedTime returntimer;
    ElapsedTime timercollect;
    public int clawintake = 1;
    public int waitfordown = 0;
    public double fisttime = 2;
    public double robotclimb = 0;
    public double isclimbing = 0;
    public StateInSample stateInSample = StateInSample.TRANSFER;
    public StateInSpeci stateInSpeci = StateInSpeci.TRANSFER;
    public StateOutSample stateOutSample = StateOutSample.DOWN;
    public StateOutSpeci sateOutSpeci = StateOutSpeci.DOWN;
    public Scoremode scoremode = Scoremode.SPECIMEN;
    double pressedretraction = 0;

    public enum StateInSpeci {
        TRANSFER,
        DOWN,
        RETRACTSERVOS,
        RETRACTEXT;
    }

    public enum StateInSample {
        TRANSFER,
        DOWN,
        RETRACTSERVOS,
        RETRACTEXT;
    }

    public enum StateOutSpeci {
        SCORING,
        DOWN;
    }

    public enum StateOutSample {
        DOWN,
        MOVEINTOTRANSFER,
        HANDSHAKE,
        EXTEND,
        EXTENDLINK,
        RELEASED;
    }

    public enum Scoremode {
        SAMPLE,
        SPECIMEN;

    }

    public SaibaControl(Gamepad gamepad1, Gamepad gamepad2, SaibaModules modules) {
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);

        this.gamepad2 = gamepad2;
        stickyGamepad2 = new StickyGamepad(gamepad2);
        outtimer = new ElapsedTime();
        intaketimer = new ElapsedTime();
        returntimer = new ElapsedTime();
        timercollect = new ElapsedTime();
        this.modules = modules;
    }

    public void EXTCONTROL() {
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.MANUAL);
            modules.EXT.setState(EXT.State.MANUAL);
            modules.EXT.setManualPower(-gamepad2.left_stick_y);
            if (modules.EXT.manualPower < 0 && (modules.EXT.getRealCurrentPosition() < 7))
                modules.EXT.setManualPower(0);
            if (modules.EXT.manualPower > 0 && (modules.EXT.getRealCurrentPosition() > 44))
                modules.EXT.setManualPower(0);
        } else {
            modules.EXT.setManualPower(0);
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
        }
        if (gamepad2.b) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
            modules.EXT.setState(EXT.State.FULL);

        }

    }

    public void LIFTCONTROL() {
        if (modules.LIFT.getRealCurrentPosition() > 20) {
            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.1) {

                modules.LIFT.setManualPower(gamepad2.right_trigger - gamepad2.left_trigger);
                if (modules.LIFT.manualPower < 0 && (modules.LIFT.getRealCurrentPosition() < 0))
                    modules.LIFT.setManualPower(0);
                if (modules.LIFT.manualPower > 0 && (modules.LIFT.getRealCurrentPosition() > 72))
                    modules.LIFT.setManualPower(0);

                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.MANUAL);
                modules.LIFT.setState(LIFT.State.MANUAL);
            } else {
                modules.LIFT.setManualPower(0);
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
            }
        }
    }

    public void SAMPLECONTROL() {
        if (stateInSample == StateInSample.TRANSFER) {
            if (gamepad1.dpad_right) {
                scoremode = Scoremode.SPECIMEN;
                stateInSpeci = StateInSpeci.TRANSFER;
                if (modules.EXT.getRealCurrentPosition() < 10) {
                    modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                    modules.EXT.setState(EXT.State.MID);
                }
                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                modules.INTUR.setState(INTUR.State.FRONT);

                modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                modules.LIFT.setState(LIFT.State.START);
                fisttime = 2;
            }
        }

        if ((gamepad2.b || stickyGamepad2.x) && stateInSample == StateInSample.TRANSFER) {
            modules.INTUR.setState(INTUR.State.FRONT);
            intaketimer.reset();
            stateInSample = StateInSample.DOWN;
            modules.INCLAW.setState(INCLAW.State.RELEASE);
            clawintake = 0;

        }
        if (stateInSample == StateInSample.DOWN) { //E la sol
            COLLECTCOMANDS();
        }
        if (modules.INTUR.getState() == INTUR.State.FRONT && intaketimer.milliseconds() > 150 && stateInSample == StateInSample.DOWN) {
            if (waitfordown != 1) {
                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);
            }
        }

        if ((gamepad2.dpad_down) && waitfordown == 0)//return
        {
            returntimer.reset();
            stateInSample = StateInSample.RETRACTSERVOS;
            if (modules.LIFT.getRealCurrentPosition() < 5) {

                modules.LINKEAGE.setState(LINKEAGE.State.TRANSFER);
                modules.OUTROTATIE.setState(OUTROTATIE.State.TRANSFERNOU);
            }
        }

        if (stateInSample == StateInSample.RETRACTSERVOS) {

            modules.INROTATOR.setState(INROTATOR.State.THIRD);
            modules.INCLAW.setState(INCLAW.State.SECURE);

            waitfordown = 0;
            returntimer.reset();
            if (modules.EXT.getRealCurrentPosition() < 20.5) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
            } else {
                modules.INBRAT.setState(INBRAT.State.TRANSFERV2);
                stateInSample = StateInSample.RETRACTEXT;
            }
        }
        if (stateInSample == StateInSample.RETRACTEXT && (Math.abs(modules.INBRAT.inbrat.profile.getPosition() - INBRAT.transferv2) <= 0.02)) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
            modules.EXT.setState(EXT.State.START);

            stateInSample = StateInSample.TRANSFER;
            returntimer.reset();
        }
//        if(stateInSample==StateInSample.TRANSFER &&modules.EXT.getRealCurrentPosition()<10 && modules.EXT.EXT.motor.motor.getCurrent(CurrentUnit.AMPS) > amp){
//            modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
//            modules.INCLAW.setState(INCLAW.State.RELEASE);
//        }
    }

    public void OuttakeControl() {
        if (scoremode == Scoremode.SPECIMEN) {


            if (gamepad2.y && modules.LIFT.getState() == LIFT.State.START) {
                modules.LIFT.setState(LIFT.State.SCORESPECItele);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECITELE);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECITELE);
                modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                outtimer.reset();

                sateOutSpeci = StateOutSpeci.SCORING;
            }
            if (sateOutSpeci == StateOutSpeci.SCORING && outtimer.milliseconds() > 100 && gamepad2.right_stick_y < -0.8) {
                modules.LINKEAGE.setState(LINKEAGE.State.OVERCENTER);
                modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                outtimer.reset();
            }
            if (sateOutSpeci == StateOutSpeci.SCORING && outtimer.milliseconds() > 200  && modules.LINKEAGE.getState()== LINKEAGE.State.OVERCENTER) {

                modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
            }


            if (sateOutSpeci == StateOutSpeci.SCORING  && (gamepad1.right_bumper || gamepad1.a)) {
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);

                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                sateOutSpeci = StateOutSpeci.DOWN;
                pressedretraction = 1;
                outtimer.reset();
            }
            if (pressedretraction == 1 && outtimer.milliseconds() > 120)
                modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
            if (pressedretraction == 1 && outtimer.milliseconds() > 220) {
                modules.LIFT.setState(LIFT.State.START);
                modules.OUTCLAW.setState(OUTCLAW.State.SECURE);

                sateOutSpeci = StateOutSpeci.DOWN;
                pressedretraction = 0;
            }


        }
        if (scoremode == Scoremode.SAMPLE) {

            if (modules.LIFT.getState() == LIFT.State.START)
                modules.LIFT.setState(LIFT.State.TRANSFER);
            if (stateOutSample == StateOutSample.DOWN && gamepad2.dpad_up) {
                modules.OUTBRAT.setState(OUTBRAT.State.TRANSFERV2);
                outtimer.reset();
                stateOutSample = StateOutSample.MOVEINTOTRANSFER;

            }
            if (outtimer.milliseconds() > 150 && stateOutSample == StateOutSample.MOVEINTOTRANSFER && (Math.abs(modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.transfnou) <= 0.02)) {
                modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                stateOutSample = StateOutSample.HANDSHAKE;

                outtimer.reset();
            }
            if (outtimer.milliseconds() > 100 && stateOutSample == StateOutSample.HANDSHAKE) {
                modules.INCLAW.setState(INCLAW.State.RELEASE);
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
                stateOutSample = StateOutSample.EXTEND;

                outtimer.reset();
            }
            if (stateOutSample == StateOutSample.EXTEND && outtimer.milliseconds() > 100) {
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
                modules.LIFT.setState(LIFT.State.FULL);
                modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                outtimer.reset();
                stateOutSample = StateOutSample.EXTENDLINK;

            }
            if (stateOutSample == StateOutSample.EXTENDLINK && outtimer.milliseconds() > 200) {
                modules.LINKEAGE.setState(LINKEAGE.State.OVERCENTER);
                modules.INBRAT.setState(INBRAT.State.UP);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
            }
            if (stateOutSample == StateOutSample.EXTENDLINK && outtimer.milliseconds() > 100 && gamepad2.a) {
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                outtimer.reset();
                stateOutSample = StateOutSample.RELEASED;
            }
            if (stateOutSample == StateOutSample.RELEASED && outtimer.milliseconds() > 100 && gamepad2.right_stick_y > 0.1) {
                modules.LINKEAGE.setState(LINKEAGE.State.TRANSFER);

                modules.OUTBRAT.setState(OUTBRAT.State.MID);
                modules.OUTROTATIE.setState(OUTROTATIE.State.TRANSFERNOU);
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
                modules.LIFT.setState(LIFT.State.TRANSFER);
                stateOutSample = StateOutSample.DOWN;
            }
            if (modules.EXT.getCurrentPosition() < modules.EXT.midpos && stateOutSample == StateOutSample.RELEASED) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
            }
        }
    }

    void SPECICONTROL() {
        if (gamepad1.dpad_left) {
            scoremode = Scoremode.SAMPLE;
            stateInSample = StateInSample.TRANSFER;
            if (modules.EXT.getRealCurrentPosition() < 10) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
            }
            modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
            modules.OUTROTATIE.setState(OUTROTATIE.State.TRANSFERNOU);
            modules.OUTBRAT.setState(OUTBRAT.State.MID);
            fisttime = 0;
        }
        if (stickyGamepad2.a) {
            if (clawintake == 1) {
                if (modules.INBRAT.getState() != INBRAT.State.MIDDLE) {
                    modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
                    modules.INCLAW.setState(INCLAW.State.RELEASE);
                    clawintake = 0;

                } else {

                    modules.INTUR.setState(INTUR.State.DIAG);
                    clawintake = 0;
                    timercollect.reset();
                    waitfordown = 2;

                }
            } else {
                if (modules.INBRAT.getState() != INBRAT.State.MIDDLE) {
                    modules.INBRAT.setState(INBRAT.State.DOWN);
                }
                timercollect.reset();
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                waitfordown = 1;

            }
        }
        if (timercollect.milliseconds() > 150 && waitfordown == 1) {


            modules.INCLAW.setState(INCLAW.State.SECURE);
            clawintake = 1;
            waitfordown = 0;
            intaketimer.reset();
        }

        if (timercollect.milliseconds() > 60 && waitfordown == 2) {
            modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
            modules.INCLAW.setState(INCLAW.State.RELEASE);
            clawintake = 0;
            waitfordown = 0;
            intaketimer.reset();
        }

        if (stickyGamepad2.x) {
            modules.INTUR.setState(INTUR.State.FRONT);
            timercollect.reset();
            modules.INROTATOR.setState(INROTATOR.State.THIRD);

            stateInSpeci = StateInSpeci.DOWN;
            modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
            modules.INCLAW.setState(INCLAW.State.RELEASE);
            clawintake = 0;

        }
        if (modules.INTUR.getState() == INTUR.State.FRONT && intaketimer.milliseconds() > 100 && stateInSpeci == StateInSpeci.DOWN) {
            if (waitfordown != 1 && timercollect.milliseconds() > 100) {
                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

            }
        }
        if (stateInSpeci == StateInSpeci.DOWN) //E LA SOL
            COLLECTWITHOUTCLAW();


        if (gamepad2.dpad_down && waitfordown != 1) {
            stateInSpeci = StateInSpeci.RETRACTSERVOS;
        }
        if (stateInSpeci == StateInSpeci.RETRACTSERVOS) {

            modules.INBRAT.setState(INBRAT.State.MIDDLE);
            modules.INROTATOR.setState(INROTATOR.State.FIRST);
            modules.INTUR.setState(INTUR.State.THROW);
            stateInSpeci = StateInSpeci.RETRACTEXT;
            returntimer.reset();

        }
        if (returntimer.milliseconds() > 100 && stateInSpeci == StateInSpeci.RETRACTEXT) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
            modules.EXT.setState(EXT.State.ZERO);


            stateInSpeci = StateInSpeci.TRANSFER;
            returntimer.reset();
        }

    }

    void COLLECTWITHOUTCLAW() {
        if (gamepad2.right_bumper) {
            if (modules.INROTATOR.rotpose < 0.78)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose + 0.06);
            if (modules.INROTATOR.rotpose > 0.78)
                modules.INROTATOR.setState(INROTATOR.State.FIFTH);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.left_bumper) {
            if (modules.INROTATOR.rotpose > 0.2)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose - 0.06);
            if (modules.INROTATOR.rotpose < 0.2)
                modules.INROTATOR.setState(INROTATOR.State.FIRST);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.y) {

            modules.INROTATOR.setState(INROTATOR.State.THIRD);
            modules.INROTATOR.setCustom(INROTATOR.trdpos);
        }


    }

    void COLLECTCOMANDS() {
        //            if(modules.EXT.getRealCurrentPosition()>15)
//                modules.INBRAT.setState(INBRAT.State.DOWNEXT);
        if (gamepad2.right_bumper) {
            if (modules.INROTATOR.rotpose < 0.78)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose + 0.06);
            if (modules.INROTATOR.rotpose > 0.78)
                modules.INROTATOR.setState(INROTATOR.State.FIFTH);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.left_bumper) {
            if (modules.INROTATOR.rotpose > 0.2)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose - 0.06);
            if (modules.INROTATOR.rotpose < 0.2)
                modules.INROTATOR.setState(INROTATOR.State.FIRST);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.y) {

            modules.INROTATOR.setState(INROTATOR.State.THIRD);
            modules.INROTATOR.setCustom(INROTATOR.trdpos);
        }


        if (stickyGamepad2.a) {
            if (clawintake == 1) {

                modules.INCLAW.setState(INCLAW.State.RELEASE);
                clawintake = 0;
            } else {
                timercollect.reset();
                modules.INBRAT.setState(INBRAT.State.DOWN);
                waitfordown = 1;

            }
        }
        if (timercollect.milliseconds() > 150 && waitfordown == 1) {
            modules.INCLAW.setState(INCLAW.State.SECURE);
            clawintake = 1;
            waitfordown = 0;
            intaketimer.reset();
        }
    }

    void CLIMB() {
//        if(modules.OUTBRAT.getState()== OUTBRAT.State.SPECIMEN )
//            modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECITELE);


        if (gamepad2.dpad_up || gamepad1.b) {
            modules.WEELIE.setState(WEELIE.State.RIDICAT);
            modules.PTO.setState(PTO.State.SLOW);
        }
        if (gamepad1.y) {
            modules.WEELIE.setState(WEELIE.State.NERIDICAT);
        }
        if (gamepad1.a) {
            modules.PTO.setState(PTO.State.FAST);
        }
        if (isclimbing == 0 || isclimbing == 6) {
            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.1) {
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.MANUAL);
                modules.LIFT.setState(LIFT.State.MANUAL);
                modules.LIFT.setManualPower(gamepad2.right_trigger - gamepad2.left_trigger);


                if (modules.LIFT.manualPower > 0 && (modules.LIFT.getRealCurrentPosition()) > 72)
                    modules.LIFT.setManualPower(0);
            } else {
                modules.LIFT.setManualPower(0);
//                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
            }
        }
//        if (gamepad2.a|| isclimbing==1)
//        {
//            isclimbing=1;
//            modules.LIFT.setRunMode(LIFT.EXTRUNMODE.MANUAL);
//            modules.LIFT.setState(LIFT.State.MANUAL);
//            modules.LIFT.setManualPower(-1);
//        }
        if (gamepad2.dpad_down) {
            isclimbing = 2;
        }
        if(gamepad1.y)
        {
            modules.WEELIE.setState(WEELIE.State.NERIDICAT);
            modules.LIFT.setState(LIFT.State.MANUAL);
            modules.LIFT.setManualPower(0);
            isclimbing=-999;
        }
        if (isclimbing == 2) {
            modules.LIFT.setState(LIFT.State.MANUAL);
            modules.LIFT.setManualPower(-1);
            modules.EXT.setState(EXT.State.MANUAL);
            modules.EXT.setManualPower(-0.3);
            if (modules.LIFT.getRealCurrentPosition() < 13 && modules.LIFT.getRealCurrentPosition() >10)  {
                modules.WEELIE.setState(WEELIE.State.NERIDICAT);
            }
            if (modules.LIFT.getRealCurrentPosition() < -0) {
                isclimbing = 3;
            }
        }
        if (isclimbing == 3) {
            modules.LIFT.setManualPower(0.3);
            if (modules.LIFT.getRealCurrentPosition() > 10) {
                modules.INBRAT.setState(INBRAT.State.UP);
                modules.LIFT.setManualPower(1);
            }
            if (modules.LIFT.getRealCurrentPosition() > 50) {
                modules.LIFT.setManualPower(0.3);
            }
            if (modules.LIFT.getRealCurrentPosition() > 56) {
                modules.LIFT.setManualPower(0);
                isclimbing = 4;
                returntimer.reset();
            }
        }
        if (isclimbing == 4) {
            modules.OUTROTATIE.setAuto(0.45);
            modules.OUTROTATIE.setState(OUTROTATIE.State.AUTOSPECI);
            modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
            modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
            robotclimb = modules.LIFT.getRealCurrentPosition();
            if (returntimer.milliseconds() > 1100 && (Math.abs(modules.OUTBRAT.outst.profile.getPosition() - OUTBRAT.specimenpos) <= 0.02)) {
                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                returntimer.reset();
                isclimbing = 5;
            }
        }
        if (isclimbing == 5) {
            if (returntimer.milliseconds() > 300) {
                if (modules.LIFT.getRealCurrentPosition() > robotclimb -2 )
                    modules.LIFT.setManualPower(-0.3);
                else {
                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
                    modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECITELE);
                    modules.LIFT.setManualPower(-0);
                    isclimbing = 6;
                    returntimer.reset();
                }
            }
        }
        if (isclimbing == 6) {
            if (returntimer.milliseconds() > 250) {
                modules.LIFT.setState(LIFT.State.MANUAL);
                modules.LIFT.setManualPower(-1);
                if (modules.LIFT.getRealCurrentPosition() < 30) {

                    modules.LINKEAGE.setState(LINKEAGE.State.OPEN);

                }

                if (modules.LIFT.getRealCurrentPosition() < 10) {
                    modules.WEELIE.setState(WEELIE.State.NERIDICAT);
                }
                if (modules.LIFT.getRealCurrentPosition() < 1) {
                    modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                    isclimbing = 7;
                }
            }
        }
    }

    @Override
    public void update() {
        if (fisttime == 0 || fisttime == 2) {
            modules.update();
            if(fisttime==2) {
                modules.INBRAT.isprofiled=0;
                modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
            }
            if(fisttime==0)
                modules.INBRAT.isprofiled=1;
            fisttime = 1;

        } else {
            modules.update();

            stickyGamepad1.update();
            stickyGamepad2.update();

            if (gamepad2.share) {
                modules.LIFT.LIFT1.resetEncoder();
                modules.LIFT.LIFT2.resetEncoder();
                modules.LIFT.setState(LIFT.State.SCORESPECItele);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECITELE);
                fisttime = 100;
            }
            if (gamepad1.options)
                fisttime = 1;
            if (fisttime == 100) {
                modules.LIFT.kI = 0;
                modules.LIFT.kIjos = 0;
                modules.EXT.setState(EXT.State.ZERO);
                CLIMB();
            } else {
                if (scoremode == Scoremode.SAMPLE)
                    SAMPLECONTROL();
                else
                    SPECICONTROL();
                OuttakeControl();
                EXTCONTROL();
                LIFTCONTROL();
            }
        }

    }
}