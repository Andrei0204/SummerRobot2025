package org.firstinspires.ftc.teamcode.roboDrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class BezTopoint extends LinearOpMode {

    SampleMecanumDrive wheels;
    AnotherMecanumDrive drive;
    SexyLocalizer localizer;
//    MisuDrivetrain drivetrainControl;
//    ElapsedTime timer;

    public static int heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer(hardwareMap, wheels);
        drive = new AnotherMecanumDrive(wheels, localizer, AnotherMecanumDrive.RunMode.PID, true);
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
//            drivetrainControl.update();
//            if (gamepad1.a)
////                drive.setBezier(new Pose2d(0, 0, 0),new Pose2d(-30, -30, 0),new Pose2d(-30, -50, 0),new Pose2d(-30, -100, 0));
            if(gamepad1.b )
//            {
                drive.setTargetPose(new Pose2d(0,0,0));
//            }
            if(gamepad1.a)
                drive.setTargetPose(new Pose2d(-70,0,0));
            if(gamepad1.x)
                drive.setTargetPose(new Pose2d(0,-70,0));
//            if(gamepad1.dpad_down)
//                drive.setTargetPose(new Pose2d(-70,-122,0));
//            if(gamepad1.dpad_up)
//                drive.setTargetPose(new Pose2d(-60,-122,40));
            telemetry.addData("x", localizer.getPoseEstimate().getX());
            telemetry.addData("y", localizer.getPoseEstimate().getY());
            telemetry.addData("poseHeading", localizer.getPoseEstimate().getHeading());
         //   telemetry.addData("pos",localizer.localizer.getWheelPositions());
//            telemetry.addData("Hz", 1.0 / timer.seconds());
//            drive.telemetry(telemetry);
//            timer.reset();


            telemetry.update();
        }


    }
}
