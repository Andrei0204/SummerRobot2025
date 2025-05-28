package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.AUTO;
import static org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift.State.DOWN;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
@Disabled

@Autonomous(name = "Auto4Speci", group = "1AutoNou")
public class Auto4Speci extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    CeresModules Modules;
    ElapsedTime sleepTimer, timer;

    State currentState = State.START;

    double heading = 0;
    public final double moduleUpdateThresMs = 30;

    int cycle = 1;


    @Override
    public void runOpMode() throws InterruptedException
    {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0,0,0));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);
        Modules = new CeresModules(hardwareMap, true, this);

        Modules.pivot.Pivot.resetEncoder();
        Modules.lift.glisLeft.resetEncoder();
        Modules.lift.glisRight.resetEncoder();

        sleepTimer = new ElapsedTime();
        timer = new ElapsedTime();

        while (opModeInInit()) {
            Modules.update();
            Modules.claw.setState(Claw.State.SECURE);
            // e deja pe pozitie
        }

        sleepTimer.startTime();
        timer.startTime();

        Localizer.setPose(new Pose2d(0,0,0));
        setNewState(State.START);

        while(opModeIsActive()){

            switch(currentState){
                case START:
                    Modules.pivotS.setAuto(0.31);
                    Modules.pivotS.setState(PivotS.State.AUTOSPECI);
                    Modules.pivot.setAuto(48 * Pivot.ticksDegree);
                    Modules.pivot.setState(Pivot.State.AUTO);
                    Modules.lift.setAuto(15.5 * Lift.ticksCm);
                    Modules.lift.setState(AUTO);
                    setNewState(State.GOTO_PRELOAD);
                    break;
                case GOTO_PRELOAD:
                    Drive.setTargetPose(PRELOAD_POSE);
                    if(Drive.reachedTargetOrStopped(2)){
                        setNewState(State.PLACE_PRELOAD);
                        sleepTimer.reset();
                    }
                    break;
                case PLACE_PRELOAD:
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    if(sleepTimer.milliseconds()>460){
                        Modules.claw.setState(Claw.State.RELEASE);
                        Modules.lift.setState(DOWN);
                        setNewState(State.GOTO_FIRSTSAMPLE);
                        sleepTimer.reset();
                    }
                    break;
                case GOTO_FIRSTSAMPLE:
                    Modules.pivot.setState(Pivot.State.MIDDLE);
                    Drive.setTargetPose(BEHIND_FST_SAMPLE);
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(MOVE_TO_FST_SAMPLE);
                        setNewState(State.GOTO_BEHIND_FIRST);
                    }
                    break;
                case GOTO_BEHIND_FIRST:
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(FRONT_SAMPLE_SCORE);
                        setNewState(State.BEFOREPUSHFIRST);
                    }

                    break;
                case BEFOREPUSHFIRST:
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(SCORE_FST_SAMPLE);
                        setNewState(State.PUSHFISRTANDGOBACK);
                    }
                    break;
                case PUSHFISRTANDGOBACK:
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(MOVE_TO_SECOND_SAMPLE);
                        setNewState(State.GOTO_THIRDSAMPLE);
                    }
                    break;
                case GOTO_THIRDSAMPLE:

                    if(Drive.reachedTarget(10)) {
                        Drive.setTargetPose(FRONT_SCND_SAMPLE);
                        setNewState(State.SCORESAMPLE);
                    }
                    break;
                case SCORESAMPLE:
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(SCORE_SCND_SAMPLE);
                        setNewState(State.GOTO_FRONTOFSPECI);
                    }
                    break;
                case GOTO_FRONTOFSPECI:
                    if(Drive.reachedTarget(3)) {
                        Drive.setTargetPose(FRONT_OF_SPECI);
                        Modules.pivot.setState(Pivot.State.WALLCOLLECT);
                        Modules.pivotS.setState(PivotS.State.WALL);
                        Modules.claw.setState(Claw.State.RELEASE);
                        Modules.lift.setState(DOWN);
                        if (Drive.reachedTargetOrStopped(4)) {
                            setNewState(State.GOTO_SPECIMEN);
                            sleepTimer.reset();
                        }
                    }
                    break;
                case GOTO_SPECIMEN:
                    if(sleepTimer.milliseconds()>100) {
                        Drive.setTargetPose(SPECIMEN_POSITION);
                        Modules.pivot.setState(Pivot.State.WALLCOLLECT);
                        Modules.pivotS.setState(PivotS.State.WALL);
                        Modules.claw.setState(Claw.State.RELEASE);
                        Modules.lift.setState(DOWN);
                        if (Drive.reachedTargetOrStopped(3)) {
                            setNewState(State.COLLECT_SPECI_AND_GOTO_BAR);
                            Modules.claw.setState(Claw.State.SECURE);
                            sleepTimer.reset();
                        }
                    }
                    break;
                case COLLECT_SPECI_AND_GOTO_BAR:
                    if(sleepTimer.milliseconds()>250){
                        Modules.pivot.setState(Pivot.State.DOWN); // up
                        Drive.setTargetPose(FRONT_OF_BAR);
                        Modules.pivotS.setState(PivotS.State.SPECIMEN);
                        if(Modules.pivot.getRealCurrentPosition()<5 )
                        {
                            Modules.lift.setState(AUTO);
                            Modules.lift.setAuto(11*Lift.ticksCm);
                        }
                        if(Modules.lift.getRealCurrentPosition()>8)
                        {
                            Modules.claw.setState(Claw.State.INTERIOR);
                            setNewState(State.PLACE);
                        }
                    }
                    break;
                case PLACE:
                    switch (cycle){
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
                            break;

                    }

                    if(Drive.reachedTargetOrStopped(2)){
                        Modules.lift.setAuto(27.5*Lift.ticksCm);
                        Modules.lift.setState(AUTO);
                        Modules.claw.setState(Claw.State.SECURE);
                        if(Modules.lift.getRealCurrentPosition()>22){
                            Modules.claw.setState(Claw.State.RELEASE);
                            cycle++;
                            if(cycle<=3)
                                setNewState(State.GOTO_FRONTOFSPECI);
                            else setNewState(State.PARK);
                        }
                    }
                    break;
                case PARK:
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

    public void setNewState(State newState){
        this.currentState = newState;
    }

    public static Pose2d PRELOAD_POSE = new Pose2d(-60, 0, 0);// todo adapt based on field
    public static Pose2d BEHIND_FST_SAMPLE = new Pose2d(-57, 72, 0);
    public static Pose2d MOVE_TO_FST_SAMPLE = new Pose2d(-127, 72, 0);
    public static Pose2d FRONT_SAMPLE_SCORE = new Pose2d(-127, 100, 0);
    public static Pose2d SCORE_FST_SAMPLE = new Pose2d(-30, 100, 0);

    public static Pose2d MOVE_TO_SECOND_SAMPLE = new Pose2d(-127, 100, 90);
    public static Pose2d FRONT_SCND_SAMPLE = new Pose2d(-127, 115, 90);
    public static Pose2d SCORE_SCND_SAMPLE = new Pose2d(-30, 115, 90);

    public static Pose2d SECOND_SAMPLE = new Pose2d(-49.5, 107, -22.35);
    public static Pose2d THIRD_SAMPLE = new Pose2d(-49.5, 131, -22.35);
    public static Pose2d DROP_THIRD_SAMPLE = new Pose2d(-27.5, 115, -150);
    public static Pose2d FRONT_OF_SPECI = new Pose2d(-27.5, 97, -180);
    public static Pose2d SPECIMEN_POSITION = new Pose2d(-13.5, 97, -180);

    public static Pose2d FRONT_OF_BAR = new Pose2d(-35, -30, -180);

    public static Pose2d PLACE_1 = new Pose2d(-57, -42, -180);
    public static Pose2d PLACE_2 = new Pose2d(-57, -36, -180);
    public static Pose2d PLACE_3 = new Pose2d(-57, -25, -180);
    public static Pose2d PLACE_4 = new Pose2d(-57, -15, -180);

    public static Pose2d PARK = new Pose2d(-20, 50, -110);

    public void updateShit(){
        Drive.update();
        Localizer.update();
        if(timer.milliseconds()>moduleUpdateThresMs){
            Modules.update();
            timer.reset();
        }
    }

    public enum State {
        START,
        GOTO_PRELOAD,
        PLACE_PRELOAD,
        GOTO_FIRSTSAMPLE,
        GOTO_BEHIND_FIRST,
        BEFOREPUSHFIRST,
        PUSHFISRTANDGOBACK,
        GOTO_THIRDSAMPLE,
        SCORESAMPLE,
        GOTO_FRONTOFSPECI,
        GOTO_SPECIMEN,
        COLLECT_SPECI_AND_GOTO_BAR,
        PLACE,
        PARK,
        END
    }

}
