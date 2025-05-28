package org.firstinspires.ftc.teamcode.robotoffseason.Modules.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;
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

public class SaibaControl implements IRobotModule {

    public final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;
    public SaibaModules modules;
    ElapsedTime outtimer;
    ElapsedTime intaketimer;
    ElapsedTime returntimer;
    ElapsedTime timercollect;
    int statescollectsample = 0;
    public int stateouttakesample = 0;
    public int stateouttakespeci = 0;
    public int specimode = 0;
    public int statecollectspeci = 0;
    public int clawintake = 1;
    public int waitfordown = 0;


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
            modules.EXT.setManualPower(-gamepad2.left_stick_y);
            if (modules.EXT.manualPower < 0 && (modules.EXT.getRealCurrentPosition()) < 3)
                modules.EXT.setManualPower(0);
            if (modules.EXT.manualPower > 0 && (modules.EXT.getRealCurrentPosition()) > 48)
                modules.EXT.setManualPower(0);
            modules.EXT.setRunMode(EXT.EXTRUNMODE.MANUAL);
            modules.EXT.setState(EXT.State.MANUAL);
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
//        if (modules.LIFT.getRealCurrentPosition() > 30) {
//            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.1) {
//
//                modules.LIFT.setManualPower(gamepad2.right_trigger - gamepad2.left_trigger);
//                if (modules.LIFT.manualPower < 0 && (modules.LIFT.getRealCurrentPosition()) < 0)
//                    modules.LIFT.setManualPower(0);
//                if (modules.LIFT.manualPower > 0 && (modules.LIFT.getRealCurrentPosition()) > 72)
//                    modules.LIFT.setManualPower(0);
//
//                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.MANUAL);
//                modules.LIFT.setState(LIFT.State.MANUAL);
//            } else {
//                modules.LIFT.setManualPower(0);
//                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
//            }
//        }
    }

    public void SAMPLECONTROL() {
        if (statescollectsample == 0) {
            if (gamepad1.dpad_right) {
                specimode = 1;
                statecollectspeci = 0;
                if (modules.EXT.getRealCurrentPosition() < 10) {
                    modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                    modules.EXT.setState(EXT.State.MID);
                }
                modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                modules.INTUR.setState(INTUR.State.FRONT);

                modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                modules.LIFT.setState(LIFT.State.START);
            }
        }
        if (statescollectsample == 0 && modules.EXT.getRealCurrentPosition() < 17)
            modules.INROTATOR.setState(INROTATOR.State.THIRD);
        if ((gamepad2.b || stickyGamepad2.x) && statescollectsample == 0) {
            modules.INTUR.setState(INTUR.State.FRONT);
            intaketimer.reset();
            statescollectsample = 1;
            modules.INCLAW.setState(INCLAW.State.RELEASE);
            clawintake = 0;

        }

        if (statescollectsample == 1) { //E la sol
            COLLECTCOMANDS();
        }
        if (modules.INTUR.getState() == INTUR.State.FRONT && intaketimer.milliseconds() > 150 && statescollectsample == 1) {
            if (waitfordown != 1) {
                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

            }
        }

        if ((gamepad2.dpad_down) && waitfordown == 0)//return
        {
            returntimer.reset();
            statescollectsample = 2;
        }

        if (statescollectsample == 2) {

            modules.INBRAT.setState(INBRAT.State.UP);
            modules.INROTATOR.setState(INROTATOR.State.FIRST);
            modules.INCLAW.setState(INCLAW.State.SEMIOPEN);
            modules.INTUR.setState(INTUR.State.TRANSFER);
            statescollectsample = 3;
            returntimer.reset();
            if (modules.EXT.getRealCurrentPosition() < 6) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.START);
            }

        }
        if (returntimer.milliseconds() > 200 && statescollectsample == 3) {

            modules.INCLAW.setState(INCLAW.State.INTERIOR);
        }
        if (returntimer.milliseconds() > 200 && statescollectsample == 3) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
            modules.EXT.setState(EXT.State.START);


            statescollectsample = 0;
            returntimer.reset();
        }
    }

    public void OuttakeControl() {
        if (specimode == 1) {


            if (gamepad2.y && modules.LIFT.getState() == LIFT.State.START) {
                modules.LIFT.setState(LIFT.State.SCORESPECI);
                modules.OUTBRAT.setState(OUTBRAT.State.SCORESPECI);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESPECI);
                outtimer.reset();
                stateouttakespeci = 1;
            }
            if (stateouttakespeci == 1 && outtimer.milliseconds() > 100 && gamepad2.right_stick_y < -0.8) {
                modules.LINKEAGE.setState(LINKEAGE.State.OPEN);
            }
            if (gamepad1.a) {
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
            }

            if (stateouttakespeci == 1 && outtimer.milliseconds() > 100 && gamepad2.a ) {
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);

                modules.LIFT.setState(LIFT.State.START);
                modules.OUTBRAT.setState(OUTBRAT.State.SPECIMEN);
                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);
                modules.OUTROTATIE.setState(OUTROTATIE.State.SPECIMEN);
                stateouttakespeci = 0;
            }

        }
        if (specimode != 1) {


            if (modules.LIFT.getState() == LIFT.State.START)
                modules.LIFT.setState(LIFT.State.TRANSFER);
            if (stateouttakesample == 0 && modules.EXT.getRealCurrentPosition() < 10 && gamepad2.dpad_up) {
                modules.OUTBRAT.setState(OUTBRAT.State.MID);
                outtimer.reset();
                stateouttakesample = -1;
            }
            if (outtimer.milliseconds() > 75 && stateouttakesample == -1) {
                modules.OUTCLAW.setState(OUTCLAW.State.INTERIOR);
                stateouttakesample = -2;
                outtimer.reset();
            }
            if (outtimer.milliseconds() > 100 && stateouttakesample == -2) {
                modules.INCLAW.setState(INCLAW.State.RELEASE);
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
                stateouttakesample = -3;
                outtimer.reset();
            }
            if (stateouttakesample == -3 && outtimer.milliseconds() > 100) {
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
                modules.LIFT.setState(LIFT.State.FULL);
                modules.OUTBRAT.setState(OUTBRAT.State.SAMPLE);
                outtimer.reset();
                stateouttakesample = -4;
            }
            if (stateouttakesample == -4 && outtimer.milliseconds() > 200)
                modules.LINKEAGE.setState(LINKEAGE.State.OVERCENTER);
            modules.OUTROTATIE.setState(OUTROTATIE.State.SCORESMAPLE);
            if (stateouttakesample == -4 && outtimer.milliseconds() > 100 && gamepad2.a) {
                modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
                outtimer.reset();
                stateouttakesample = -5;
            }
            if (stateouttakesample == -5 && outtimer.milliseconds() > 100 && gamepad2.right_stick_y > 0.1) {
                modules.LINKEAGE.setState(LINKEAGE.State.CLOSED);

                modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);
                modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
                modules.LIFT.setRunMode(LIFT.EXTRUNMODE.PID);
                modules.LIFT.setState(LIFT.State.TRANSFER);
                stateouttakesample = 0;
            }
            if (modules.EXT.getCurrentPosition() < modules.EXT.midpos && stateouttakesample < -4) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
            }
        }
    }

    void SPECICONTROL() {
        if (gamepad1.dpad_left) {
            specimode = 0;
            statescollectsample = 0;
            if (modules.EXT.getRealCurrentPosition() < 10) {
                modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
                modules.EXT.setState(EXT.State.MID);
            }
            modules.OUTROTATIE.setState(OUTROTATIE.State.COLLECT);
            modules.OUTBRAT.setState(OUTBRAT.State.TRANSFER);

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

                waitfordown = 1;

            }
        }
        if (timercollect.milliseconds() > 150 && waitfordown == 1) {
            modules.OUTCLAW.setState(OUTCLAW.State.RELEASE);
            modules.INCLAW.setState(INCLAW.State.INTERIOR);
            clawintake = 1;
            waitfordown = 0;
            intaketimer.reset();
        }

        if (timercollect.milliseconds() > 30 && waitfordown == 2) {
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

            statecollectspeci = 1;
            modules.OUTCLAW.setState(OUTCLAW.State.SECURE);
            modules.INCLAW.setState(INCLAW.State.RELEASE);
            clawintake = 0;

        }
        if (modules.INTUR.getState() == INTUR.State.FRONT && intaketimer.milliseconds() > 100 && statecollectspeci == 1) {
            if (waitfordown != 1 && timercollect.milliseconds() > 100) {
                modules.INBRAT.setState(INBRAT.State.SEMIDOWN);

            }
        }
        if (statecollectspeci == 1) //E LA SOL
            COLLECTWITHOUTCLAW();


        if (gamepad2.dpad_down && waitfordown != 1) {
            statecollectspeci = 2;
        }
        if (statecollectspeci == 2) {

            modules.INBRAT.setState(INBRAT.State.MIDDLE);
            modules.INROTATOR.setState(INROTATOR.State.FIRST);
            modules.INTUR.setState(INTUR.State.THROW);
            statecollectspeci = 3;
            returntimer.reset();

        }
        if (returntimer.milliseconds() > 100 && statecollectspeci == 3) {
            modules.EXT.setRunMode(EXT.EXTRUNMODE.PID);
            modules.EXT.setState(EXT.State.ZERO);


            statecollectspeci = 0;
            returntimer.reset();
        }

    }

    void COLLECTWITHOUTCLAW() {
        if (gamepad2.right_bumper) {
            if (modules.INROTATOR.rotpose < 0.78)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose + 0.03);
            if (modules.INROTATOR.rotpose > 0.78)
                modules.INROTATOR.setState(INROTATOR.State.FIFTH);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.left_bumper) {
            if (modules.INROTATOR.rotpose > 0.2)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose - 0.03);
            if (modules.INROTATOR.rotpose < 0.2)
                modules.INROTATOR.setState(INROTATOR.State.FIRST);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.y) {

            modules.INROTATOR.setState(INROTATOR.State.THIRD);
            modules.INROTATOR.setCustom(Rotator.trdpos);
        }


    }

    void COLLECTCOMANDS() {
        //            if(modules.EXT.getRealCurrentPosition()>15)
//                modules.INBRAT.setState(INBRAT.State.DOWNEXT);
        if (gamepad2.right_bumper) {
            if (modules.INROTATOR.rotpose < 0.78)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose + 0.03);
            if (modules.INROTATOR.rotpose > 0.78)
                modules.INROTATOR.setState(INROTATOR.State.FIFTH);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.left_bumper) {
            if (modules.INROTATOR.rotpose > 0.2)
                modules.INROTATOR.setCustom(modules.INROTATOR.rotpose - 0.03);
            if (modules.INROTATOR.rotpose < 0.2)
                modules.INROTATOR.setState(INROTATOR.State.FIRST);
            else
                modules.INROTATOR.setState(INROTATOR.State.CUSTOM);


        }
        if (gamepad2.y) {

            modules.INROTATOR.setState(INROTATOR.State.THIRD);
            modules.INROTATOR.setCustom(Rotator.trdpos);
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
            modules.INCLAW.setState(INCLAW.State.INTERIOR);
            clawintake = 1;
            waitfordown = 0;
            intaketimer.reset();
        }
    }

    void CLIMB() {
        if (gamepad1.b) {
            modules.WEELIE.setState(WEELIE.State.RIDICAT);
        }
        if (gamepad1.y) {
            modules.WEELIE.setState(WEELIE.State.NERIDICAT);
        }
        if (gamepad1.x) {
            modules.PTO.setState(PTO.State.SLOW);
        }

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


    @Override
    public void update() {

//CLIMB();
        stickyGamepad1.update();
        stickyGamepad2.update();

        EXTCONTROL();
        LIFTCONTROL();
        if (specimode == 0)
            SAMPLECONTROL();
        else
            SPECICONTROL();
        OuttakeControl();

    }
}