package org.firstinspires.ftc.teamcode.roboDrive.Auto2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.AnotherMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.SexyLocalizer;

@Config
@TeleOp
public class BezTopoint2 extends LinearOpMode {

    SampleMecanumDrive wheels;
    AnotherMecanumDrive2 drive;
    SexyLocalizer2 localizer;
//    MisuDrivetrain drivetrainControl;
//    ElapsedTime timer;

    public static int heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer2(hardwareMap, wheels);
        drive = new AnotherMecanumDrive2(wheels, localizer, AnotherMecanumDrive2.RunMode.PID, true);
//        drivetrainControl = new MisuDrivetrain(gamepad1, drive);
//        timer = new ElapsedTime();
//        localizer.setPose(new Pose2d(0, 0, 0));
//        timer.startTime();
//        drive.setLineBezier1(localizer.getPoseEstimate(), localizer.getPoseEstimate());

        heading = 0;

        waitForStart();


        while (opModeIsActive()) {
            drive.update();
            localizer.update();
            if(gamepad1.b )
                drive.setTargetPose(new Pose2d(0,0,0));
            if(gamepad1.dpad_up )
                drive.setTargetPose(new Pose2d(0,0,90));
            if(gamepad1.dpad_down )
                drive.setTargetPose(new Pose2d(0,0,-90));
            if(gamepad1.a)
                drive.setTargetPose(new Pose2d(-70,0,0));
            if(gamepad1.x)
                drive.setTargetPose(new Pose2d(0,-70,0));

            telemetry.addData("x", localizer.getPoseEstimate().getX());
            telemetry.addData("y", localizer.getPoseEstimate().getY());
            telemetry.addData("poseHeading", localizer.getPoseEstimate().getHeading());
            telemetry.addData("pos",localizer.localizer.getWheelPositions());



            telemetry.update();
        }


    }
}
