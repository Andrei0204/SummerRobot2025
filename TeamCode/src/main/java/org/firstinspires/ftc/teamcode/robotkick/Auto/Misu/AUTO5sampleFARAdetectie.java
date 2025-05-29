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
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;

@Autonomous(name = "AUTO5sampleFARAdetectie", group = "1AutoNou")
public class AUTO5sampleFARAdetectie extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    CeresModules Modules;
    ElapsedTime sleepTimer, timer,timeout;

    State currentState = State.START;

    double heading = 0;
    public final double moduleUpdateThresMs = 30;

    int cycle = 1;

    public static double forwardOffset = 0;
    public double lateralOffset = 0;
    public static int roteisan = 2;
    private boolean collectedOnce = false;


    @Override
    public void runOpMode() throws InterruptedException
    {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0,0,Math.PI/2));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);
        Modules = new CeresModules(hardwareMap, true, this);

        Modules.pivot.Pivot.resetEncoder();
        Modules.lift.glisLeft.resetEncoder();
        Modules.lift.glisRight.resetEncoder();

        sleepTimer = new ElapsedTime();
        timer = new ElapsedTime();
        timeout= new ElapsedTime();

        while (opModeInInit()) {
            Modules.update();
            Modules.claw.setState(Claw.State.SECURE);

            if(gamepad1.dpad_up){
                forwardOffset+=1;
                while (gamepad1.dpad_up && !isStopRequested());
            }

            if(gamepad1.dpad_down){
                forwardOffset-=1;
                while (gamepad1.dpad_down && !isStopRequested());
            }

            if(gamepad1.dpad_right){
                lateralOffset-=1;
                while (gamepad1.dpad_right && !isStopRequested());
            }

            if(gamepad1.dpad_left){
                lateralOffset+=1;
                while (gamepad1.dpad_left && !isStopRequested());
            }

            if(gamepad1.a){
                roteisan = Math.max(0, roteisan-1);
                while (gamepad1.a && !isStopRequested());
            }

            if(gamepad1.y){
                roteisan = Math.min(4, roteisan+1);
                while (gamepad1.y && !isStopRequested());
            }



            telemetry.addData("forwardOffset", forwardOffset);
            telemetry.addData("lateralOffset", lateralOffset);
            telemetry.addData("roteisan", roteisan);


            telemetry.update();


            // e deja pe pozitie
        }

        sleepTimer.startTime();
        timer.startTime();
        timeout.startTime();

        Localizer.setPose(new Pose2d(0,0,Math.PI/2));
        setNewState(State.START);
        Modules.pivot.max = 0.9   ;
        COLLECT_SUB = new Pose2d(133.5+forwardOffset, -20, 90);
        while(opModeIsActive()){
            switch (currentState){
                case START:
                    setNewState(State.GOTO_BASKET);
                    timeout.reset();
                    break;
                case GOTO_BASKET:
                    Drive.setTargetPose(BASKET);
                    if(cycle>=4)
                    {
                        Drive.lateralMultiplier =1.3;
                    }
                    if(Drive.reachedTarget(20))
                        Modules.pivot.setState(Pivot.State.DOWN);
                    Modules.rotator.setState(Rotator.State.THIRD);
                    if(Modules.pivot.getRealCurrentPosition()<5 && Drive.reachedTarget(15)) {
                        Modules.lift.setState(SECOND);
                        Modules.pivotS.setState(PivotS.State.WALL);
                        Modules.claw.setState(Claw.State.INTERIOR);
                    }
                    if(Modules.lift.getRealCurrentPosition()>62 && (Drive.reachedTargetOrStopped(4) || timeout.milliseconds()>2500)){
                        Modules.pivotS.setState(PivotS.State.SCORE);
                        Drive.lateralMultiplier =0.65;
                        timeout.reset();
                        setNewState(State.RELEASE);
                        sleepTimer.reset();
                    }
                    break;
                case RELEASE:
                    if(sleepTimer.milliseconds()>400) {
                        Modules.claw.setState(Claw.State.RELEASE);
                        sleepTimer.reset();
                        setNewState(State.GODOWN);
                    }
                    break;
                case GODOWN:
                    if(sleepTimer.milliseconds()>250) {
                        Modules.lift.setState(DOWN);
                        Modules.pivotS.setState(PivotS.State.DOWN);
                        if (Modules.lift.getRealCurrentPosition() < 3) {
                            Modules.pivot.setState(Pivot.State.MIDDLE);
                            if (Modules.pivot.getRealCurrentPosition() > 70) {
                                setNewState(State.GOTO_SAMPLE);
                                timeout.reset();
                            }
                            if(collectedOnce){
                                setNewState(State.END);
                            }
                        }
                    }
                    break;
                case GOTO_SAMPLE:
                    switch (cycle){
                        case 1:
                            Drive.setTargetPose(FIRST_SAMPLE);
                            Modules.lift.setAuto(25 * Lift.ticksCm);
                            Modules.lift.setState(AUTO);
                            break;
                        case 2:
                            Drive.setTargetPose(SECOND_SAMPLE);
                            Modules.lift.setAuto(25 * Lift.ticksCm);
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
                            setNewState(State.END);
                            break;
                    }
                    if((Drive.reachedTargetOrStopped(2)|| timeout.milliseconds()>2500) && Drive.reachedHeading(2) && Modules.lift.getRealCurrentPosition()>23.5 && cycle < 4){
                        cycle++;
                        Modules.claw.setState(Claw.State.SECURE);
                        sleepTimer.reset();
                        setNewState(State.COLLECT_AND_PLACE_TAPE);
                    }
                    break;
                case COLLECT_AND_PLACE_TAPE:
                    if(sleepTimer.milliseconds()>350){
                        Modules.lift.setState(DOWN);
                        Modules.pivotS.setState(PivotS.State.WALL);
                        if(Modules.lift.getRealCurrentPosition()<5){
                            if(cycle<5)
                                setNewState(State.GOTO_BASKET);
                            else setNewState(State.END);
                        }
                    }
                    break;
                case GOTO_INTER_SUB:
                    Modules.lift.setState(DOWN);
                    Modules.claw.setState(Claw.State.RELEASE);
                    Modules.pivotS.setState(PivotS.State.DOWN);
                    if(Modules.lift.getRealCurrentPosition()<3){
                        Modules.pivot.setState(Pivot.State.MIDDLE);
                        Drive.setTargetPose(INTER_COLLECT_SUB);
                        if(Drive.reachedTargetOrStopped(4) || timeout.milliseconds()>2000){
                            timeout.reset();
                            setNewState(State.GOTO_COLLECT_SUB);
                        }
                    }
                    break;
                case GOTO_COLLECT_SUB:
                    Drive.setMaxPower(0.5);
                    Drive.setTargetPose(COLLECT_SUB);
                    Modules.lift.setAuto((15+lateralOffset)*Lift.ticksCm);
                    Modules.lift.setState(AUTO);
                    switch (roteisan){
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

                    if((Drive.reachedTargetOrStopped(3)||timeout.milliseconds()>2000) && Modules.lift.getRealCurrentPosition()>12){
                        Modules.claw.setState(Claw.State.SECURE);
                        sleepTimer.reset();
                        setNewState(State.GET_OUTOF_SUB);
                        collectedOnce = true;
                        Drive.setMaxPower(1);
                    }
                    break;
                case GET_OUTOF_SUB:
                    if(sleepTimer.milliseconds()>300) {
                        Modules.pivotS.setState(PivotS.State.WALL);
                        Drive.setMaxPower(1);
                        Drive.setTargetPose(new Pose2d(INTER_COLLECT_SUB.getX(), INTER_COLLECT_SUB.getY(), 140));
                        if (Drive.reachedTarget(15)) {
                            Drive.setMaxPower(1);
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

            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("Heading", Localizer.getPose().getHeading());

            telemetry.addData("Cycle", cycle);
            telemetry.addData("Reached Target", Drive.reachedTarget(2));
            telemetry.addData("Reached Heading", Drive.reachedHeading(1));

            telemetry.addData("Reached Target and Stop", Drive.reachedTargetAndStopped(2));
            telemetry.addData("Reached Heading and Stop", Drive.reachedHeadingAndStopped(1));

            telemetry.addData("Current State", currentState.toString());

//            Modules.telemetry(telemetry);
            telemetry.update();

        }

    }

    public void setNewState(State newState){
        this.currentState = newState;
    }


    public static Pose2d BASKET = new Pose2d(28, 53 , 135);// todo adapt based on field
    public static Pose2d FIRST_SAMPLE = new Pose2d(50, 65.6, 168);
    public static Pose2d SECOND_SAMPLE = new Pose2d(48, 80, 180);
    public static Pose2d THIRD_SAMPLE = new Pose2d(49, 87.6, 202);
    public static Pose2d INTER_COLLECT_SUB = new Pose2d(133.5, 28.5, 90);
    public static Pose2d INTER_COLLECT_SUB_leave = new Pose2d(133.5, 27, 135);
    public static Pose2d COLLECT_SUB = new Pose2d(133.5+forwardOffset, -20, 90);
    public static Pose2d PLACE = new Pose2d(-57, -42, -180);

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
        GOTO_BASKET,
        RELEASE,
        GODOWN,
        GOTO_SAMPLE,
        COLLECT_AND_PLACE_TAPE,
        GOTO_INTER_SUB,
        GET_OUTOF_SUB,
        GOTO_COLLECT_SUB,
        PARK,
        END
    }

}
