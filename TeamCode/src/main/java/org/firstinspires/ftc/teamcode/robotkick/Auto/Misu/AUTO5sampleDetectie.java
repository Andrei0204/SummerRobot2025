package org.firstinspires.ftc.teamcode.robotkick.Auto.Misu;

import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.AUTO;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.DOWN;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.SECOND;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingold.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.Limelight;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;

@Autonomous(name = "AUTO5sampleDetectie", group = "1AutoNou")

public class AUTO5sampleDetectie extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    CeresModules Modules;
    Limelight limelight;
    ElapsedTime sleepTimer, timer, timeout;
StickyGamepad Gamepad1;
    State currentState = State.START;

    double heading = 0;
    public final double moduleUpdateThresMs = 30;

    int cycle = 1;
    int detection = 0;

    public static double forwardOffset = 0;
    public double lateralOffset = 0;
    public static int roteisan = 2;
    private boolean collectedOnce = false;

    Pose2d location = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);
        Modules = new CeresModules(hardwareMap, true, this);
        limelight = new Limelight(hardwareMap);
        Modules.pivot.Pivot.resetEncoder();
        Modules.lift.glisLeft.resetEncoder();
        Modules.lift.glisRight.resetEncoder();

        sleepTimer = new ElapsedTime();
        timer = new ElapsedTime();
        timeout = new ElapsedTime();
        limelight.blue_yllw_nopython();
        Gamepad1=new StickyGamepad(gamepad1);
        while (opModeInInit()) {
            Modules.update();
            Modules.claw.setState(Claw.State.SECURE);

            if (gamepad1.dpad_up) {
                forwardOffset += 1;
                while (gamepad1.dpad_up && !isStopRequested()) ;
            }

            if (gamepad1.dpad_down) {
                forwardOffset -= 1;
                while (gamepad1.dpad_down && !isStopRequested()) ;
            }

            if (gamepad1.dpad_right) {
                lateralOffset -= 1;
                while (gamepad1.dpad_right && !isStopRequested()) ;
            }

            if (gamepad1.dpad_left) {
                lateralOffset += 1;
                while (gamepad1.dpad_left && !isStopRequested()) ;
            }

            if (Gamepad1.left_bumper) {
                roteisan = Math.max(0, roteisan - 1);
                while (gamepad1.a && !isStopRequested()) ;
            }

            if (Gamepad1.right_bumper) {
                roteisan = Math.min(4, roteisan + 1);
                while (gamepad1.y && !isStopRequested()) ;
            }
            if (gamepad1.a) {
                detection = 0;
                //limelight.blue_yllw_nopython();
            }
            if (gamepad1.b) {
                detection = 3;
                //  limelight.red_nopython();
            }
            if (gamepad1.x) {
                detection = 2;
                //   limelight.blue_nopython();
            }


            telemetry.addData("forwardOffset", forwardOffset);
            telemetry.addData("lateralOffset", lateralOffset);
            telemetry.addData("roteisan", roteisan);
            telemetry.addData("Limelight", detection);
            telemetry.addData("Limelight",  limelight.limelight.getStatus().getPipelineIndex());
            telemetry.addLine("0 galben, 2 albastru,3 rosu");

            Gamepad1.update();
            telemetry.update();


            // e deja pe pozitie
        }

        sleepTimer.startTime();
        timer.startTime();
        timeout.startTime();

        Localizer.setPose(new Pose2d(0, 0, Math.PI / 2));
        setNewState(State.START);
        Modules.pivot.max = 0.9;
        COLLECT_SUB = new Pose2d(133.5 + forwardOffset * 2, -20, 90);
        INTER_COLLECT_SUB = new Pose2d(133.5+ forwardOffset * 2, 28.5, 90);
        while (opModeIsActive()) {
            switch (currentState) {
                case START:
                    setNewState(State.GOTO_BASKET);
                    timeout.reset();
                    break;
                case GOTO_BASKET:
                    Drive.setTargetPose(BASKET);
                    if (cycle >= 4) {
                        Drive.lateralMultiplier = 1.3;
                    }
                    if (Drive.reachedTarget(20))
                        Modules.pivot.setState(Pivot.State.DOWN);
                    Modules.rotator.setState(Rotator.State.SECOND);
                    if (Modules.pivot.getRealCurrentPosition() < 5 && Drive.reachedTarget(15)) {
                        Modules.lift.setState(SECOND);
                        Modules.pivotS.setState(PivotS.State.WALL);
                        Modules.claw.setState(Claw.State.INTERIOR);
                    }
                    if (Modules.lift.getRealCurrentPosition() > 62 && (Drive.reachedTargetOrStopped(4) || timeout.milliseconds() > 2500)) {
                        Modules.pivotS.setState(PivotS.State.SCORE);
                        Drive.lateralMultiplier = 0.65;
                        timeout.reset();
                        Drive.setMaxPower(1);
                        setNewState(State.RELEASE);
                        sleepTimer.reset();
                    }
                    break;
                case RELEASE:
                    if (sleepTimer.milliseconds() > 400) {
                        Modules.claw.setState(Claw.State.RELEASE);
                        sleepTimer.reset();
                        setNewState(State.GODOWN);
                    }
                    break;
                case GODOWN:
                    if (sleepTimer.milliseconds() > 250) {
                        Modules.lift.setState(DOWN);
                        Modules.rotator.setState(Rotator.State.THIRD);
                        Modules.pivotS.setState(PivotS.State.DOWN);

                        switch (cycle) {
                            case 1:
                                Drive.setTargetPose(FIRST_SAMPLE);
                                break;
                            case 2:
                                Drive.setTargetPose(SECOND_SAMPLE);

                                break;
                            case 3:
                                Drive.setTargetPose(THIRD_SAMPLE);

                                break;
//                            case 4:
//                                setNewState(State.GOTO_INTER_SUB);
//                                timeout.reset();
//                                break;
//                            case 5:
//                                setNewState(State.END);
//                                break;
                        }
                        if (Modules.lift.getRealCurrentPosition() < 3) {
                            Modules.pivot.setState(Pivot.State.MIDDLE);
                            if (Modules.pivot.getRealCurrentPosition() > 70) {
                                setNewState(State.GOTO_SAMPLE);
                                timeout.reset();
                            }
                            if (collectedOnce) {
                                setNewState(State.GOTO_SAMPLE);
                                cycle++;
                            }
                        }
                    }
                    break;
                case GOTO_SAMPLE:
                    switch (cycle) {
                        case 1:
                            Drive.setTargetPose(FIRST_SAMPLE);
                            Modules.lift.setAuto(23 * Lift.ticksCm);
                            Modules.lift.setState(AUTO);
                            break;
                        case 2:
                            Drive.setTargetPose(SECOND_SAMPLE);
                            Modules.lift.setAuto(23 * Lift.ticksCm);
                            Modules.lift.setState(AUTO);
                            break;
                        case 3:
                            Drive.setTargetPose(THIRD_SAMPLE);
                            Modules.lift.setAuto(25 * Lift.ticksCm);
                            Modules.lift.setState(AUTO);
                            break;
                        case 4:
                            setNewState(State.GOTO_INTER_SUB);
                            timeout.reset();
                            break;
                        case 5:

                            setNewState(State.GOTO_INTER_SUB);
                            timeout.reset();
                            break;
                    }
                    if ((Drive.reachedTargetOrStopped(2) || timeout.milliseconds() > 2000) && Drive.reachedHeading(2) && Modules.lift.getRealCurrentPosition() > 23.5 && cycle < 4) {
                        timeout.reset();
                        timer.reset();
                        sleepTimer.reset();
                        setNewState(State.DETECTSAMPLE);
                    }
                    break;
                case DETECTSAMPLE:
                    if (cycle <= 2) {
                        if (location.getY() == 0 && location.getX() == 0) {
                            if (cycle == 1)
                                location = limelight.getdistanceVector(168);
                            else
                                location = limelight.getdistanceVector(180);
                        }
                        else if (location.getY() != 0 && location.getX() != 0) {
                            if (cycle == 1)
                                Drive.setTargetPose(new Pose2d(Localizer.getCurrentPosition().getX() + location.getX() - 1, Localizer.getCurrentPosition().getY() + location.getY(), 168));
                            else
                                Drive.setTargetPose(new Pose2d(Localizer.getCurrentPosition().getX() + location.getX() - 1, Localizer.getCurrentPosition().getY() + location.getY(), 180));


                            timeout.reset();
                            timer.reset();
                            sleepTimer.reset();
                            setNewState(State.COLLECT_AND_PLACE_TAPE);
                            cycle++;

                        }
                        if (timeout.milliseconds() > 400 && (location.getY() == 0 && location.getX() == 0)) {
                            timeout.reset();
                            timer.reset();
                            sleepTimer.reset();
                            setNewState(State.COLLECT_AND_PLACE_TAPE);
                            cycle++;
                        }
                    } else {
                        timeout.reset();
                        timer.reset();
                        sleepTimer.reset();
                        setNewState(State.COLLECT_AND_PLACE_TAPE);
                        cycle++;
                    }

                    break;


                case COLLECT_AND_PLACE_TAPE:
                    if ((Drive.reachedTargetOrStopped(2) || timeout.milliseconds() > 2500) && Drive.reachedHeading(2) || cycle >= 4) {
                        Modules.claw.setState(Claw.State.SECURE);
                        location = new Pose2d(0, 0, 0);
                        if (sleepTimer.milliseconds() > 350) {

                            if (cycle == 5)
                                Modules.lift.setState(DOWN);
                            Modules.pivot.setState(Pivot.State.DOWN);
                            Modules.pivotS.setState(PivotS.State.WALL);
                            if (Modules.lift.getRealCurrentPosition() < 60) {
                                if (cycle < 5) {
                                    timeout.reset();
                                    timer.reset();
                                    sleepTimer.reset();
                                    setNewState(State.GOTO_BASKET);

                                } else {
                                    timeout.reset();
                                    timer.reset();
                                    sleepTimer.reset();
                                    setNewState(State.END);
                                }
                            }
                        }
                    } else
                        sleepTimer.reset();
                    break;
                case GOTO_INTER_SUB:
                    Modules.lift.setState(DOWN);
                    Modules.claw.setState(Claw.State.RELEASE);
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    if (Modules.lift.getRealCurrentPosition() < 3) {
                        Modules.pivot.setState(Pivot.State.MIDDLE);
                        Drive.setTargetPose(INTER_COLLECT_SUB);
                        if (Drive.reachedTargetOrStopped(4) || timeout.milliseconds() > 2000) {
                            timeout.reset();
                            setNewState(State.GOTO_COLLECT_SUB);
                        }
                    }
                    break;
                case GOTO_COLLECT_SUB:
                    Drive.setMaxPower(0.5);
                    Drive.setTargetPose(COLLECT_SUB);
                    if (cycle <= 4) {
                        Modules.lift.setAuto((15 + lateralOffset) * Lift.ticksCm);
                        Modules.lift.setState(AUTO);
                        if ((Drive.reachedTargetOrStopped(3) || timeout.milliseconds() > 2000) && Modules.lift.getRealCurrentPosition() > 12) {
                            //DETECTIE
                            timer.reset();
                            Drive.setMaxPower(1);
                            sleepTimer.reset();
                            if (detection == 0)
                                limelight.blue_yllw_nopython();
                            else if (detection == 2)
                                limelight.blue_nopython();
                            else
                                limelight.red_nopython();
                            setNewState(State.DETECT);
                            timer.reset();
                            timeout.reset();

                        }
                    } else //PARCAREEEEEEE
                        if ((Drive.reachedTargetOrStopped(3))) {
//                            Modules.lift.setAuto(13 * Lift.ticksCm);
//                            Modules.pivot.setAuto(40 * Pivot.ticksDegree);
//                            Modules.pivot.setState(Pivot.State.AUTO);
//                            Modules.lift.setState(Lift.State.AUTO);
                        }

                    break;
                case DETECT:
                    if (location.getY() == 0 && location.getX() == 0)
                        location = limelight.getdistanceVector(90);
                    else if (location.getY() != 0 && location.getX() != 0) {
                        //  drive.Lime = true;
                        AnotherMecanumDrivePedroPP.translationalPID.p = 0.11;
                        Drive.setTargetPose(new Pose2d(Localizer.getCurrentPosition().getX() + location.getX(), Localizer.getCurrentPosition().getY(), 90));
                        switch (roteisan) {
                            case 0:
                                Modules.rotator.setState(Rotator.State.FIRST);
                                break;
                            case 1:
                                Modules.rotator.setState(Rotator.State.SECOND);
                                break;
                            case 2:
                                Modules.rotator.setState(Rotator.State.THIRD);
                                break;
                            case 3:
                                Modules.rotator.setState(Rotator.State.FOURTH);
                                break;
                            case 4:
                                Modules.rotator.setState(Rotator.State.FIFTH);
                                break;
                        }
                        if (Modules.lift.getRealCurrentPosition() - location.getY() < 30)
                            Modules.lift.setAuto((Modules.lift.getRealCurrentPosition() - location.getY()) * Lift.ticksCm);
                        Modules.lift.setState(AUTO);
                        setNewState(State.COLLECT_SUB);
                    }
                    if (timeout.milliseconds() > 500 && (location.getY() == 0 && location.getX() == 0))
                        setNewState(State.COLLECT_SUB);
                    break;
                case COLLECT_SUB:
                    if ((Drive.reachedTargetAndStopped(3)) &&
                            Modules.lift.getRealCurrentPosition() - 1 < (Modules.lift.getTarget() / Lift.ticksCm) &&
                            (Modules.lift.getTarget() / Lift.ticksCm) < Modules.lift.getRealCurrentPosition() + 1) {
                        AnotherMecanumDrivePedroPP.translationalPID.p = 0.05;
                        Drive.setMaxPower(1);
                        Modules.claw.setState(Claw.State.SECURE);
                        sleepTimer.reset();
                        setNewState(State.GET_OUTOF_SUB);
                        collectedOnce = true;
                        timeout.reset();
                    }
                    break;

                case GET_OUTOF_SUB:
                    if (sleepTimer.milliseconds() > 300) {

                        Modules.pivotS.setState(PivotS.State.WALL);
                        Modules.lift.setState(DOWN);
                        Drive.setMaxPower(1);

                        Drive.setTargetPose(new Pose2d(INTER_COLLECT_SUB.getX(), INTER_COLLECT_SUB.getY(), 140));
                        if (Drive.reachedTarget(15)) {
                            Drive.setMaxPower(0.9);
                            setNewState(State.COLLECT_AND_PLACE_TAPE);

                        }
                    }
                    break;
                case END:
                    updateShit();
                    break;
            }
            Drive.update();
            Localizer.update();
            Modules.update();

//            telemetry.addData("X", Localizer.getCurrentPosition().getX());
//            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
//            telemetry.addData("Heading", Localizer.getPose().getHeading());
//
//            telemetry.addData("Cycle", cycle);
//            telemetry.addData("Reached Target", Drive.reachedTarget(2));
//            telemetry.addData("Reached Heading", Drive.reachedHeading(1));
//
//            telemetry.addData("Reached Target and Stop", Drive.reachedTargetAndStopped(2));
//            telemetry.addData("Reached Heading and Stop", Drive.reachedHeadingAndStopped(1));

            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("locatiom", location);
            telemetry.addData("cyclu", cycle);
            telemetry.addData("Collect_sub", COLLECT_SUB.getX());
            telemetry.addData("Limelight",  limelight.limelight.getStatus().getPipelineIndex());
            telemetry.addData("MaxPow",  Drive.maxpower);


//            Modules.telemetry(telemetry);
            telemetry.update();

        }

    }

    public void setNewState(State newState) {
        this.currentState = newState;
    }

    public void detection(double heading) {
        if (location.getY() == 0 && location.getX() == 0)
            location = limelight.getdistanceVector(heading);
        else if (location.getY() != 0 && location.getX() != 0) {
            Drive.setTargetPose(new Pose2d(Localizer.getCurrentPosition().getX() + location.getX() - 1, Localizer.getCurrentPosition().getY() + location.getY(), heading));
            timeout.reset();
            sleepTimer.reset();
        } else if (timeout.milliseconds() > 1000) {
            timeout.reset();
            sleepTimer.reset();
        }
    }


    public static Pose2d BASKET = new Pose2d(28, 53, 135);// todo adapt based on field
    public static Pose2d FIRST_SAMPLE = new Pose2d(48, 65.6, 168);
    public static Pose2d SECOND_SAMPLE = new Pose2d(46, 79, 180);
    public static Pose2d THIRD_SAMPLE = new Pose2d(50, 87.6, 200);
    public static Pose2d INTER_COLLECT_SUB = new Pose2d(133.5+ forwardOffset * 2, 28.5, 90);
    public static Pose2d INTER_COLLECT_SUB_leave = new Pose2d(133.5, 27, 135);
    public static Pose2d COLLECT_SUB = new Pose2d(133.5 + forwardOffset * 2, -20, 90);
    public static Pose2d PLACE = new Pose2d(-57, -42, -180);

    // public static Pose2d PARK = new Pose2d(-20, 50, -110);

    public void updateShit() {
        Drive.update();
        Localizer.update();
        if (timer.milliseconds() > moduleUpdateThresMs) {
            Modules.update();
            timer.reset();
        }
    }

    public enum State {
        START,
        GOTO_BASKET,
        RELEASE,
        GODOWN,
        GOTO_SAMPLE,
        COLLECT_AND_PLACE_TAPE,
        GOTO_INTER_SUB,
        GET_OUTOF_SUB,
        COLLECT_SUB,
        DETECT,
        DETECTSAMPLE,
        GOTO_COLLECT_SUB,
        PARK,
        END
    }

}
