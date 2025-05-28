package org.firstinspires.ftc.teamcode.robotkick.Modules.drive;

import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS.State.SCORE;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS.State.SPECIMEN;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS.State.WALL;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

public class CeresControl implements IRobotModule {

    public final StickyGamepad stickyGamepad1, stickyGamepad2;
    private final Gamepad gamepad1, gamepad2;
    public CeresModules modules;

    public boolean left, right;
    double clawpos = 0;
    int rotatorpos = 3;
    double entrypos = 0;
    Boolean canextend = false;
    ElapsedTime timer;
    ElapsedTime timerglis;
    boolean liftbool = false;
    int rotation=0;

    public CeresControl(Gamepad gamepad1, Gamepad gamepad2, CeresModules modules) {
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);

        this.gamepad2 = gamepad2;
        stickyGamepad2 = new StickyGamepad(gamepad2);
        timer = new ElapsedTime();
        timerglis = new ElapsedTime();
        this.modules = modules;
    }

    public void ServoControl() {

        if (stickyGamepad2.a) {

            if (clawpos == 0) {
                clawpos = 1;
                if (modules.pivot.getState() == Pivot.State.WALLCOLLECT)
                    modules.claw.setState(Claw.State.INTERIOR);
                else
                    modules.claw.setState(Claw.State.SECURE);
            } else {
                clawpos = 0;
                modules.claw.setState(Claw.State.RELEASE);
                if (modules.pivotS.getState() == SCORE)
//                    timer.startTime();
                    timer.reset();
            }
        }
        if (timer.milliseconds() > 200 && modules.pivotS.getState() == SCORE && modules.claw.getState() == Claw.State.RELEASE) {
            modules.pivotS.setState(PivotS.State.DOWN);
            modules.rotator.setState(Rotator.State.THIRD);
            modules.rotator.setCustom(Rotator.trdpos);
            timer.reset();
        }

        if (modules.pivot.getState() == Pivot.State.DOWN && stickyGamepad2.y) {
            rotation=0;
            modules.pivotS.setState(PivotS.State.SPECIMEN);
            modules.lift.setState(Lift.State.SPECIMEN);
        } else if (modules.pivot.getState() == Pivot.State.MIDDLE || modules.pivot.getState() == Pivot.State.PICKUP)
            modules.pivotS.setState(PivotS.State.DOWN);
        else if (modules.pivot.getState() == Pivot.State.ENTRY) {
            modules.pivotS.setState(PivotS.State.DIAGONAL);
        } else if (modules.pivot.getState() == Pivot.State.WALLCOLLECT) {
            modules.pivotS.setState(PivotS.State.WALL);
        } else {
            if (modules.claw.getState() == Claw.State.SECURE && modules.pivotS.getState() != PivotS.State.SPECIMEN)
                modules.pivotS.setState(SCORE);
        }


//        if (stickyGamepad2.right_bumper) {
//            rotatorpos++;
//            if (rotatorpos > 5)
//                rotatorpos = 5;
//
//            if (rotatorpos == 6)
//                rotatorpos = 3;
//        }
//        if (stickyGamepad2.left_bumper) {
//            rotatorpos--;
//            if (rotatorpos < 1)
//                rotatorpos = 1;
//            if (rotatorpos == 6)
//                rotatorpos = 3;
//        }
//        if (stickyGamepad2.b)
//            rotatorpos = 6;
        if (modules.pivotS.PivState == PivotS.State.DOWN || modules.pivotS.PivState == PivotS.State.DIAGONAL) {
            if (gamepad2.right_bumper) {
                if (modules.rotator.rotpose < 0.74)
                    modules.rotator.setCustom(modules.rotator.rotpose + 0.025);
                if (modules.rotator.rotpose > 0.74)
                    modules.rotator.setState(Rotator.State.FIFTH);
                else
                    modules.rotator.setState(Rotator.State.CUSTOM);


            }
            if (gamepad2.left_bumper) {
                if (modules.rotator.rotpose > 0.175)
                    modules.rotator.setCustom(modules.rotator.rotpose - 0.025);
                if (modules.rotator.rotpose < 0.175)
                    modules.rotator.setState(Rotator.State.FIRST);
                else
                    modules.rotator.setState(Rotator.State.CUSTOM);


            }
            if (gamepad2.x) {

                modules.rotator.setState(Rotator.State.THIRD);
                modules.rotator.setCustom(Rotator.trdpos);
            }


            if (modules.rotator.getState() == Rotator.State.SPECIMEN) {
                modules.rotator.setState(Rotator.State.THIRD);
                modules.rotator.setCustom(Rotator.trdpos);
            }

        }

//            switch (rotatorpos) {
//                case 1:
//                    modules.rotator.setState(Rotator.State.FIRST);
//                    break;
//                case 2:
//                    modules.rotator.setState(Rotator.State.SECOND);
//                    break;
//                case 3:
//                    modules.rotator.setState(Rotator.State.THIRD);
//                    break;
//                case 4:
//                    modules.rotator.setState(Rotator.State.FOURTH);
//                    break;
//                case 5:
//                    modules.rotator.setState(Rotator.State.FIFTH);
//                    break;
//                case 6:
//                    modules.rotator.setState(Rotator.State.SPECIMEN);
//                    break;
//          }
        else if (modules.pivotS.PivState == SCORE || modules.pivotS.PivState == PivotS.State.SPECIMEN) {
            if(stickyGamepad2.left_bumper)
                rotation=1;
            else if(stickyGamepad2.right_bumper)
                rotation=0;
            if(modules.pivotS.PivState == PivotS.State.SPECIMEN)
                rotation=0;
            if(rotation==0)
            {
                modules.rotator.setState(Rotator.State.THIRD);
            }
            else
                modules.rotator.setState(Rotator.State.SECOND);


        } else
            modules.rotator.setState(Rotator.State.THIRD);
    }

    public void liftControl() {
        if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.1) {
            if(modules.pivotS.getState()==SPECIMEN)
            {
                modules.claw.setState(Claw.State.SECURE);
            }
            modules.lift.setManualPower(gamepad2.right_trigger - gamepad2.left_trigger);
            if (modules.lift.manualPower < 0 && (modules.lift.glisRight.getCurrentPosition() / Lift.ticksCm) < 0)
                modules.lift.setManualPower(0);
            if (modules.lift.manualPower > 0 && (modules.lift.glisRight.getCurrentPosition() / Lift.ticksCm) > 67.7)
                modules.lift.setManualPower(0);
            if (modules.lift.manualPower > 0 && (modules.lift.glisRight.getCurrentPosition() / Lift.ticksCm) > 29 && modules.pivot.state != Pivot.State.DOWN)
                modules.lift.setManualPower(0);
            modules.lift.setRunMode(Lift.LiftRunMode.MANUAL);
            modules.lift.setState(Lift.State.MANUAL);
        } else {
            modules.lift.setManualPower(0);
            modules.lift.setRunMode(Lift.LiftRunMode.PID);
        }
        if (stickyGamepad2.dpad_left) {
            if (modules.pivot.getState() == Pivot.State.DOWN) {
                modules.lift.setRunMode(Lift.LiftRunMode.PID);
                modules.lift.setState(Lift.State.FIRST);
            }

        }
        if (stickyGamepad2.dpad_right) {
            if (modules.pivot.getState() == Pivot.State.DOWN) {
                modules.lift.setRunMode(Lift.LiftRunMode.PID);
                modules.lift.setState(Lift.State.SECOND);
            }
        }
    }

    public void pivotControl() {

        if (stickyGamepad2.dpad_up) {
            if (modules.pivotS.getState() != SCORE) {
                if (modules.pivot.state == Pivot.State.MIDDLE) {
                    modules.pivot.setState(Pivot.State.ENTRY);
                    entrypos = 1;
                    liftbool = true;
                    timerglis.reset();
                }
                if (!liftbool && modules.pivot.state != Pivot.State.MIDDLE) {
                    modules.lift.setRunMode(Lift.LiftRunMode.PID);
                    modules.lift.setState(Lift.State.DOWN);
                }
                if (modules.lift.getRealCurrentPosition() <=7) {
                    modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
                    modules.pivot.setState(Pivot.State.DOWN);
                }
            }
        }
        if (stickyGamepad2.dpad_down) {
            if (modules.pivotS.getState() != SCORE) {
                if (modules.pivot.state == Pivot.State.MIDDLE) {
                    modules.pivot.setState(Pivot.State.ENTRY);
                    entrypos = 1;
                    liftbool = true;
                    timerglis.reset();
                }
                if (!liftbool && modules.pivot.state != Pivot.State.MIDDLE) {
                    modules.lift.setRunMode(Lift.LiftRunMode.PID);
                    modules.lift.setState(Lift.State.DOWN);
                }

                if (modules.lift.getRealCurrentPosition() <= 3) {
                    modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
                    modules.pivot.setState(Pivot.State.MIDDLE);
                }
            }
        }

        if (liftbool && timerglis.milliseconds() > 400) {
            liftbool = false;
            modules.lift.setRunMode(Lift.LiftRunMode.PID);
            modules.lift.setState(Lift.State.DOWN);

        }

        if (stickyGamepad2.b) {
            if (modules.pivot.getState() == Pivot.State.DOWN && modules.lift.getRealCurrentPosition() > 16) {
            } else {
                if (modules.lift.getRealCurrentPosition() < 3) {
                    //   modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
                    modules.pivot.setState(Pivot.State.WALLCOLLECT);
                } else {
                    modules.lift.setRunMode(Lift.LiftRunMode.PID);
                    modules.lift.setState(Lift.State.DOWN);
                }
            }
        }

        if (stickyGamepad1.x) {
            if (entrypos == 0) {
                modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
                modules.pivot.setState(Pivot.State.ENTRY);
                entrypos = 1;
            } else {
                if (modules.pivot.getState() != Pivot.State.DOWN) {
                    modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
                    modules.pivot.setState(Pivot.State.MIDDLE);
                    entrypos = 0;
                }
            }
        }
        if (gamepad2.share) {
            modules.pivot.setState(Pivot.State.PICKUP);
            modules.lift.setAuto(9);
            modules.lift.setState(Lift.State.AUTO);
        }
        if(gamepad1.a){
            modules.pivot.setRunMode(Pivot.PivotRunmode.PID);
            modules.pivot.setState(Pivot.State.DOWN);
        }
    }


    public void miscControl() {
//        if(Math.abs(gamepad2.left_stick_y)>0.1)
//        {
//            modules.pivot.setManualPower(gamepad2.left_stick_y);
//            modules.pivot.setRunMode(Pivot.PivotRunmode.MANUAL);
//        }
////        if (gamepad1.share || gamepad1.options) {
////            if (gamepad1.share)
////                modules.worm.setManualPower(1);
////            if (gamepad1.options)
////                modules.worm.setManualPower(-1);
////            modules.worm.setRunMode(Worm.WormRunMode.MANUAL);
//         else {
//            modules.pivot.setManualPower(0);
//            modules.pivot.setRunMode(Pivot.PivotRunmode.MANUAL);
//        }
    }


    @Override
    public void update() {

        stickyGamepad1.update();
        stickyGamepad2.update();
        ServoControl();
        pivotControl();
        liftControl();
        miscControl();


    }
}