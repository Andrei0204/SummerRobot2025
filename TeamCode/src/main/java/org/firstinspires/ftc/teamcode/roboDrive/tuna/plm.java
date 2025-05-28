package org.firstinspires.ftc.teamcode.roboDrive.tuna;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedro;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.SexyLocalizerPedro;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.MisuDrivetrainPedro;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.drive.CeresControl;


@TeleOp
public class plm extends LinearOpMode {


    CeresModules modules;
    SampleMecanumDrive wheels;
    SexyLocalizerPedro localizer;

    AnotherMecanumDrivePedro drive;
    ElapsedTime timer;
    ElapsedTime sleepTimer;
    MisuDrivetrainPedro drivetrainControl;
    CeresControl modulesControl;

    public enum RunMode {
        STAY,
        BACK,
        FORWARD
    }

    RunMode currentRunMode = RunMode.STAY;
    RunMode lastRunMode;

    public final double sleepTime = 1500;


    @Override
    public void runOpMode() throws InterruptedException {

        modules = new CeresModules(hardwareMap, false, this);
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizerPedro(hardwareMap,wheels);
        drive = new AnotherMecanumDrivePedro(wheels, localizer, AnotherMecanumDrivePedro.RunMode.PID, true);
        drivetrainControl = new MisuDrivetrainPedro(gamepad1, drive,modules.lift,modules.pivot);
        modulesControl = new CeresControl(gamepad1, gamepad2, modules);
        localizer.setPose(new Pose2d(0, 0, 0));
        localizer.localizer.resetIMU();
        timer = new ElapsedTime();
        sleepTimer = new ElapsedTime();
        waitForStart();
        currentRunMode = RunMode.FORWARD;
        Pose2d targetPose = new Pose2d(0,0,0);





        while (opModeIsActive()) { // singurul while

            switch(currentRunMode) {
                case STAY:
                    break;
                case FORWARD:
                    targetPose = new Pose2d(-30, 0, 0);
                    break;
                case BACK:
                    targetPose = new Pose2d(0,0,0);
                    break;
            }



            drive.setTargetPose(targetPose);
            if(drive.reachedTargetOrStopped(0.5)){
                sleepTimer.reset();
                while(sleepTimer.milliseconds()<sleepTime){
                    drive.update();
                    localizer.update();
                }
                if(currentRunMode == RunMode.FORWARD) currentRunMode = RunMode.BACK;
                else if (currentRunMode == RunMode.BACK) currentRunMode = RunMode.FORWARD;
            }

            if(gamepad1.a){
                lastRunMode = currentRunMode;
                currentRunMode = RunMode.STAY;
                while(gamepad1.a){
                    drive.update();
                    localizer.update();
                }
                currentRunMode = lastRunMode;
            }



//            modulesControl.update();
            drivetrainControl.update();
            drive.update();
            localizer.update();
//            modules.update();


            telemetry.addData("x", localizer.getPoseEstimate().getX());
//            telemetry.addData("x", localizer.localizer.getWheelPositions());
            telemetry.addData("headin", localizer.getPoseEstimate().getHeading());
            telemetry.addData("y", localizer.getPoseEstimate().getY());
            telemetry.addData("Hz", 1/timer.seconds());
            telemetry.addData("Velocity", localizer.getVelocity().getMagnitude());
            timer.reset();
            modules.telemetry(telemetry);
            telemetry.update();

        }

    }
}


