package org.firstinspires.ftc.teamcode.robotkick.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.roboDrive.AnotherMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.SexyLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.MisuDrivetrain;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.drive.CeresControl;


@TeleOp
@Disabled
public class Ceres extends LinearOpMode {


    CeresModules modules;
    SampleMecanumDrive wheels;
    SexyLocalizer localizer;

    AnotherMecanumDrive drive;
    ElapsedTime timer;
    MisuDrivetrain drivetrainControl;
    CeresControl modulesControl;

    @Override
    public void runOpMode() throws InterruptedException {

        modules = new CeresModules(hardwareMap, false, this);
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer(hardwareMap,wheels);
        drive = new AnotherMecanumDrive(wheels, localizer, AnotherMecanumDrive.RunMode.VectorAndSlows, true);
        drivetrainControl = new MisuDrivetrain(gamepad1, drive,modules.lift,modules.pivot);
        modulesControl = new CeresControl(gamepad1, gamepad2, modules);
        localizer.setPose(new Pose2d(0, 0, 0));
        timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) { // singurul while


            modulesControl.update();
            drivetrainControl.update();
            drive.update();
            //localizer.update();
            modules.update();


//            telemetry.addData("x", localizer.getPoseEstimate().getX());
//            telemetry.addData("x", localizer.localizer.getWheelPositions());
//            telemetry.addData("headin", localizer.getPoseEstimate().getHeading());
//            telemetry.addData("y", localizer.getPoseEstimate().getY());
            modules.telemetry(telemetry);
            telemetry.addData("Hz", 1/timer.seconds());
            timer.reset();
            modules.telemetry(telemetry);
            telemetry.update();

        }

    }
}


