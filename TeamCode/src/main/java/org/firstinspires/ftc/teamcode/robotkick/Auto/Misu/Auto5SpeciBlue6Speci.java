package org.firstinspires.ftc.teamcode.robotkick.Auto.Misu;

import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.AUTO;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.DOWN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingold.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;

@Autonomous(name = "Auto5SpeciBlue6Speci", group = "1AutoNou")
public class Auto5SpeciBlue6Speci extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    CeresModules Modules;
    ElapsedTime sleepTimer, timer, timeout;

    State currentState = State.START;

    double heading = 0;
    public final double moduleUpdateThresMs = 30;

    int cycle = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);
        Modules = new CeresModules(hardwareMap, true, this);

        Modules.pivot.Pivot.resetEncoder();
        Modules.lift.glisLeft.resetEncoder();
        Modules.lift.glisRight.resetEncoder();

        sleepTimer = new ElapsedTime();
        timer = new ElapsedTime();
        timeout = new ElapsedTime();

        while (opModeInInit()) {
            Modules.update();
            Modules.claw.setState(Claw.State.SECURE);
            // e deja pe pozitie
        }

        sleepTimer.startTime();
        timer.startTime();
        timeout.startTime();

        Localizer.setPose(new Pose2d(0, 0, 0));
        setNewState(State.START);

        while (opModeIsActive()) {

            switch (currentState) {
                case START:
                    Modules.pivotS.setAuto(0.31);
                    Modules.pivotS.setState(PivotS.State.AUTOSPECI);
                    Modules.pivot.setAuto(48 * Pivot.ticksDegree);
                    Modules.pivot.setState(Pivot.State.AUTO);
                    Modules.lift.setAuto(15.5 * Lift.ticksCm);
                    Modules.lift.setState(AUTO);
                    timeout.reset();
                    setNewState(State.GOTO_PRELOAD);
                    break;
                case GOTO_PRELOAD:
                    Drive.setTargetPose(PRELOAD_POSE);
                    if (Drive.reachedTargetOrStopped(4) || timeout.milliseconds() > 1500) {
                        setNewState(State.PLACE_PRELOAD);
                        sleepTimer.reset();
                    }
                    break;
                case PLACE_PRELOAD:
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    if (sleepTimer.milliseconds() > 100) {
                        Modules.claw.setState(Claw.State.RELEASE);
                        Modules.lift.setState(DOWN);


                        setNewState(State.INSUB);
                        sleepTimer.reset();
                        timeout.reset();
                    }
                    break;
                case INSUB:
                    if(Modules.lift.getRealCurrentPosition()<5)
                        Modules.pivot.setState(Pivot.State.MIDDLE);
                    if(Modules.pivot.getRealCurrentPosition()>80){

                        Modules.lift.setAuto(20*Lift.ticksCm);
                        Modules.lift.setState(AUTO);
                        if(Modules.lift.getRealCurrentPosition()>15.2)
                        {
                            sleepTimer.reset();
                            Modules.claw.setState(Claw.State.INTERIOR);
                            while(sleepTimer.milliseconds()<200){
                                updateShit();
                            }
                            Modules.pivotS.setState(PivotS.State.DIAGONAL);
                            Modules.lift.setState(DOWN);
                            setNewState(State.TOHUMAN);
                        }
                    }
                    break;
                case TOHUMAN:
                    Drive.lateralMultiplier = 1.5;
                    Drive.headingMultiplier = 3;
                    Drive.setMaxPower(0.8);

                    Drive.setTargetPose(TOHUMAN);
                    if(Drive.getLocalizer().getCurrentPosition().getY()>50){
                        Modules.pivot.setState(Pivot.State.DOWN);
                        Modules.pivotS.setState(PivotS.State.SCORE);
                        Modules.pivot.max=0.7;
                        if(Drive.reachedTarget(5))
                        {
                            Modules.claw.setState(Claw.State.RELEASE);
                            sleepTimer.reset();
                            while(sleepTimer.milliseconds()<200){
                                updateShit();
                            }
                            Drive.setTargetPose(FIRST_SAMPLE);
                            Modules.pivot.setState(Pivot.State.MIDDLE);
                            Modules.pivotS.setState(PivotS.State.DOWN);
                            timeout.reset();
                            Drive.lateralMultiplier = 0.65;
                            Drive.headingMultiplier = 1.5;
                            Drive.setMaxPower(1);
                            setNewState(State.GOTO_FIRSTSAMPLE);
                            Modules.pivot.max=1;
                        }
                    }
                    break;
                case GOTO_FIRSTSAMPLE:
                    Drive.setTargetPose(FIRST_SAMPLE);
                    Modules.pivot.setState(Pivot.State.MIDDLE);
                    if (Modules.pivot.getRealCurrentPosition()>60) {
                        Modules.lift.setState(AUTO);
                        Modules.lift.setAuto(25 * Lift.ticksCm);
                        Modules.rotator.setState(Rotator.State.THIRD);
                    }
                    if ((Drive.reachedTargetOrStopped(3     )&& Modules.lift.getRealCurrentPosition()>23 && Drive.reachedHeading(Math.toRadians(3.5)) || timeout.milliseconds() > 2700)) {

                        setNewState(State.COLLECT_FIRST_AND_PLACE);
                        sleepTimer.reset();
                        timeout.reset();
                    }
                    break;
                case COLLECT_FIRST_AND_PLACE:
                    Modules.claw.setState(Claw.State.SECURE);

                    if (sleepTimer.milliseconds() > 200) {
                        Drive.setTargetPose(new Pose2d(FIRST_SAMPLE.getX() + 8, FIRST_SAMPLE.getY()-20, -120));
                        if (Drive.reachedHeading(Math.toRadians(10)) || timeout.milliseconds() > 1200) {
                            Modules.claw.setState(Claw.State.RELEASE);
                            setNewState(State.GOTO_SECONDSAMPLE);
                            sleepTimer.reset();
                            timeout.reset();
                        }
                    }
                    break;
                case GOTO_SECONDSAMPLE:
                    Drive.setTargetPose(SECOND_SAMPLE);
                    Modules.pivot.setState(Pivot.State.MIDDLE);
//                    if(Localizer.getCurrentPosition().getY()>35){
//                        Modules.lift.setState(AUTO);
//                        Modules.lift.setAuto(25 * Lift.ticksCm);
//                        Modules.rotator.setState(Rotator.State.THIRD);
//                    }
                    if ((Drive.reachedTargetOrStopped(3.5) && Drive.reachedHeadingAndStopped(Math.toRadians(3))) || timeout.milliseconds() > 2200) {
                        setNewState(State.COLLECT_SECOND_AND_PLACE);
                        sleepTimer.reset();
                        timeout.reset();
                    }
                    break;
                case COLLECT_SECOND_AND_PLACE:
                    Modules.claw.setState(Claw.State.SECURE);
                    if (sleepTimer.milliseconds() > 250) {
                        Drive.setTargetPose(new Pose2d(FIRST_SAMPLE.getX() + 8, FIRST_SAMPLE.getY(), -150));
                        if (Drive.reachedHeading(Math.toRadians(10)) || timeout.milliseconds() > 1200) {
                            Modules.claw.setState(Claw.State.RELEASE);
                            setNewState(State.GOTO_THIRDSAMPLE);
                            Modules.lift.setState(DOWN);
                            timeout.reset();
                            sleepTimer.reset();
                        }
                    }
                    break;
                case GOTO_THIRDSAMPLE:
                    Drive.setTargetPose(THIRD_SAMPLE);
                    Modules.pivot.setState(Pivot.State.MIDDLE);
                    if (Math.toDegrees(Drive.getLocalizer().getHeading()) > -70) {
                        Modules.lift.setState(AUTO);
                        Modules.lift.setAuto(25 * Lift.ticksCm);
                    }
                    if ((Drive.reachedTargetOrStopped(3) && Drive.reachedHeadingAndStopped(Math.toRadians(5)) || timeout.milliseconds() > 3200)) {

                        Modules.lift.setState(AUTO);
                        Modules.lift.setAuto(25 * Lift.ticksCm);
                        if (Modules.lift.getRealCurrentPosition() > 24) {
                            setNewState(State.COLLECT_THIRD_AND_PLACE);
                            timeout.reset();
                            sleepTimer.reset();
                        }

                    }
                    break;
                case COLLECT_THIRD_AND_PLACE:
                    Modules.claw.setState(Claw.State.SECURE);
                    if (sleepTimer.milliseconds() > 250) {
                        Modules.lift.setAuto(11 * Lift.ticksCm);
                        Modules.rotator.setState(Rotator.State.SECOND);
                        Modules.lift.setState(AUTO);
                        Drive.setTargetPose(DROP_THIRD_SAMPLE);
                        if (Drive.reachedHeading(Math.toRadians(5)) || timeout.milliseconds() > 3500) {
                            Modules.claw.setState(Claw.State.RELEASE);
                            Modules.lift.setState(DOWN);

                            setNewState(State.GOTO_SPECIMEN);
                            sleepTimer.reset();
                            timeout.reset();
                        }
                    }
                    break;
                case GOTO_FRONTOFSPECI:
                    Drive.setTargetPose(FRONT_OF_SPECI);
//                    Modules.pivot.setState(Pivot.State.WALLCOLLECT);
//                    Modules.pivotS.setState(PivotS.State.WALL);

                    Modules.pivot.setAuto(48 * Pivot.ticksDegree);
                    Modules.pivot.setState(Pivot.State.AUTO);
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    Modules.claw.setState(Claw.State.RELEASE);
                    Modules.lift.setState(DOWN);
                    if (Drive.reachedTargetOrStopped(4) || (timeout.milliseconds() > 2300 && cycle == 1)) {
                        setNewState(State.GOTO_SPECIMEN);
                        sleepTimer.reset();
                        timeout.reset();
                    }
                    break;

                case GOTO_SPECIMEN:

                    Modules.rotator.setState(Rotator.State.THIRD);
                    Drive.setTargetPose(SPECIMEN_POSITION);
//                        Modules.pivot.setState(Pivot.State.WALLCOLLECT);
//                        Modules.pivotS.setState(PivotS.State.WALL);
                    Modules.claw.setState(Claw.State.RELEASE);
                    Modules.pivot.setAuto(51 * Pivot.ticksDegree);
                    Modules.pivot.setState(Pivot.State.AUTO);
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    Modules.lift.setState(DOWN);
                    //      Drive.setMaxPower(1);
                    if (cycle != 1)
                        Drive.lateralMultiplier = 1.3;
                    if (cycle == 1) {
                        Drive.lateralMultiplier = 1.3;
                        Modules.pivot.setAuto(49 * Pivot.ticksDegree);
                        Modules.pivot.setState(Pivot.State.AUTO);
                    }
                    if (Drive.reachedTargetOrStopped(3) || (timeout.milliseconds() > 1400 && cycle == 1)) {
                        setNewState(State.COLLECT_SPECI_AND_GOTO_BAR);
                        Modules.claw.setState(Claw.State.SECURE);
                        Drive.lateralMultiplier = 0.65;

                        Drive.setMaxPower(1);
                        timeout.reset();
                        sleepTimer.reset();
                    }
                    break;
                case COLLECT_SPECI_AND_GOTO_BAR:
                    if (sleepTimer.milliseconds() > 200) {
                        Modules.pivot.setState(Pivot.State.DOWN); // up
                        Drive.setTargetPose(FRONT_OF_BAR);

                        Modules.pivotS.setState(PivotS.State.SPECIMEN);
                        if (Modules.pivot.getRealCurrentPosition() < 5) {
                            Modules.pivotS.setState(PivotS.State.SPECIMEN);
                            Modules.lift.setAuto(15 * Lift.ticksCm);
                            Modules.lift.setState(AUTO);
                        }
                        if (Modules.lift.getRealCurrentPosition() > 9 && Drive.getLocalizer().getCurrentPosition().getY() < 30) {
                            Modules.claw.setState(Claw.State.SECURE);
                            setNewState(State.PLACE);
                        }
                    }
                    break;
                case PLACE:
                    switch (cycle) {
                        case 1:
                            Drive.setTargetPose(PLACE_1);
                            break;
                        case 2:
                            Drive.setTargetPose(PLACE_2);
                            break;
                        case 3:
                            Drive.setTargetPose(PLACE_3);
                            break;
                        case 4:
                            Drive.setTargetPose(PLACE_4);
                            break;
                        case 5:
                            Drive.setTargetPose(PLACE_5);
                            break;
                        case 6:

                            break;

                    }

                    if (Drive.reachedTargetOrStopped(6)) {
//                        Modules.lift.setAuto(27.5*Lift.ticksCm);
//                        Modules.lift.setState(AUTO);
                        Modules.claw.setState(Claw.State.SECURE);
                        if (Modules.lift.getRealCurrentPosition() > 9) {
                            Modules.claw.setState(Claw.State.RELEASE);
                            cycle++;
                            if (cycle <= 5) {

                                setNewState(State.GOTO_SPECIMEN);
                            }
                            else setNewState(State.PICKUP);
                        }
                    }
                    break;
                case PICKUP:
                    Drive.setTargetPose(PARK);
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    Modules.lift.setState(DOWN);
                    if(Modules.lift.getRealCurrentPosition()<5)
                    {
                        Modules.pivot.setState(Pivot.State.MIDDLE);
                        setNewState(State.END);
                    }
                    break;
                case END:
                    if(Modules.pivot.getRealCurrentPosition()>90) {
                        Modules.lift.setState(AUTO);
                        Modules.lift.setAuto(25*Lift.ticksCm);
                    }

                    break;
                case ENDFR:
                    updateShit();
                break;
            }




            Drive.update();
            Localizer.update();
            Modules.update();

            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("Heading", Localizer.getPose().getHeading());

            telemetry.addData("Current State", currentState.toString());

            Modules.telemetry(telemetry);

//            telemetry.addData("Frequency", 1.0/diffTime);
//            telemetry.addData("pp Frequency", Localizer.getFrequency());
            telemetry.update();
        }

    }

    public void setNewState(State newState) {
        this.currentState = newState;
    }

    public static Pose2d PRELOAD_POSE = new Pose2d(-63, -12, 0);// todo adapt based on field
    public static Pose2d FIRST_SAMPLE = new Pose2d(-46, 80, -22.35);
    public static Pose2d SECOND_SAMPLE = new Pose2d(-47, 104, -22.35);
    public static Pose2d THIRD_SAMPLE = new Pose2d(-47, 131, -22.35);
    public static Pose2d DROP_THIRD_SAMPLE = new Pose2d(-20, 100, -145);
    public static Pose2d TOHUMAN = new Pose2d(-23, 75, 60);
    public static Pose2d FRONT_OF_SPECI = new Pose2d(-29, 77, -180);
    public static Pose2d SPECIMEN_POSITION = new Pose2d(2, 69, -180);

    public static Pose2d FRONT_OF_BAR = new Pose2d(-45, -30, -180);

    public static Pose2d PLACE_1 = new Pose2d(-57, -7, -180);
    public static Pose2d PLACE_2 = new Pose2d(-59, -10, -180);
    public static Pose2d PLACE_3 = new Pose2d(-62, -15, -180);
    public static Pose2d PLACE_4 = new Pose2d(-62, -13, -180);
    public static Pose2d PLACE_5 = new Pose2d(-61, -10, -180);

//    public static Pose2d PLACE_1 = new Pose2d(-54, -14, -180);
//    public static Pose2d PLACE_2 = new Pose2d(-54, -14, -180);
//    public static Pose2d PLACE_3 = new Pose2d(-54, -14, -180);
//    public static Pose2d PLACE_4 = new Pose2d(-55, -12, -180);
//    public static Pose2d PLACE_5 = new Pose2d(-55, -10, -180);

    public static Pose2d PARK = new Pose2d(-20, 50, -110);
    public static Pose2d BASKET = new Pose2d(-11, -145, -60);
    public static Pose2d SAMPLE = new Pose2d(-40, -140, 0);
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
        INSUB,
        GOTO_PRELOAD,
        PLACE_PRELOAD,
        TOHUMAN,
        GOTO_FIRSTSAMPLE,
        COLLECT_FIRST_AND_PLACE,
        GOTO_SECONDSAMPLE,
        COLLECT_SECOND_AND_PLACE,
        GOTO_THIRDSAMPLE,
        COLLECT_THIRD_AND_PLACE,
        GOTO_FRONTOFSPECI,
        GOTO_SPECIMEN,
        COLLECT_SPECI_AND_GOTO_BAR,
        PLACE,
        PICKUP,
        COLLECTSAMPLE,
        GOTOBASKET,
        SCOREBASKET,
        END,
        ENDFR
    }

}
