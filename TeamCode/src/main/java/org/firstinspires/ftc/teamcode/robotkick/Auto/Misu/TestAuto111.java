package org.firstinspires.ftc.teamcode.robotkick.Auto.Misu;

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

@Autonomous(name = "TestAuto111", group = "1AutoNou")

public class TestAuto111 extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);

        sleepTimer = new ElapsedTime();
        timer = new ElapsedTime();
        timeout = new ElapsedTime();



        waitForStart();

        sleepTimer.startTime();
        timer.startTime();
        timeout.startTime();

        Localizer.setPose(new Pose2d(0, 0, 0));
        setNewState(State.START);

        while (opModeIsActive()){

            switch (currentState)
            {
                case START:
                    setNewState(State.GOTO_POS1);
                    break;
                case GOTO_POS1:
                    Drive.setTargetPose(POS1);
                    if(Drive.reachedTargetAndStopped(3) && Drive.reachedHeading(0.1)){
                        setNewState(State.AT_POS1);
                    }
                    break;
                case AT_POS1:
                    if(gamepad1.a)
                        setNewState(State.GOTO_POS2);
                    break;
                case GOTO_POS2:
                    Drive.setTargetPose(POS2);
                    if(Drive.reachedTargetAndStopped(3) && Drive.reachedHeading(0.1)){
                        setNewState(State.AT_POS2);
                    }
                    break;
                case AT_POS2:
                    if(gamepad1.a)
                        setNewState(State.GOTO_POS1);
                    break;
            }
            Drive.update();
            Localizer.update();


            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("H", Localizer.getCurrentPosition().getHeading());
            telemetry.update();
        }



    }

    public void setNewState(State newState) {
        this.currentState = newState;
    }

    // public static Pose2d PARK = new Pose2d(-20, 50, -110);

    public void updateShit() {
        Drive.update();
        Localizer.update();
        if (timer.milliseconds() > moduleUpdateThresMs) {
            Modules.update();
            timer.reset();
        }
    }

    Pose2d POS1 = new Pose2d(0,0,0);
    Pose2d POS2 = new Pose2d(150, 0, 90);

    public enum State {
        START,
        GOTO_POS1,
        AT_POS1,
        GOTO_POS2,
        AT_POS2,
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
