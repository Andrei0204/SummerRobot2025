package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

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

@Disabled
@Autonomous(name = "Tester", group = "1AutoNou")
public class AutoSpeciTester extends LinearOpMode {

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






            Localizer.update();


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
    public static Pose2d FIRST_SAMPLE = new Pose2d(-49.5, 81, -22.35);
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
        COLLECT_FIRST_AND_PLACE,
        GOTO_SECONDSAMPLE,
        COLLECT_SECOND_AND_PLACE,
        GOTO_THIRDSAMPLE,
        COLLECT_THIRD_AND_PLACE,
        GOTO_FRONTOFSPECI,
        GOTO_SPECIMEN,
        COLLECT_SPECI_AND_GOTO_BAR,
        PLACE,
        PARK,
        END
    }

}
