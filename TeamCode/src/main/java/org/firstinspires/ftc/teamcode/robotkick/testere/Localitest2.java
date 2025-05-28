package org.firstinspires.ftc.teamcode.robotkick.testere;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.SexyLocalizer2Imu;

@Config
@TeleOp
public class Localitest2 extends LinearOpMode {

    SampleMecanumDrive wheels;
//    AnotherMecanumDrive drive;
SexyLocalizer2Imu localizer;
//    MisuDrivetrain drivetrainControl;
//    ElapsedTime timer;

    public static int heading = 0;

    public DistanceSensor sensorDistanceback;
    public DistanceSensor sensorDistancefront;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistanceback = hardwareMap.get(DistanceSensor.class, "DS");
        sensorDistancefront = hardwareMap.get(DistanceSensor.class, "DSfront");
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer2Imu(hardwareMap, wheels);
//?        drive = new AnotherMecanumDrive(wheels, localizer, AnotherMecanumDrive.RunMode.PID, true);
////        drivetrainControl = new MisuDrivetrain(gamepad1, drive);
//        timer = new ElapsedTime();
//        localizer.setPose(new Pose2d(0, 0, 0));
//        timer.startTime();
//        drive.setLineBezier1(localizer.getPoseEstimate(), localizer.getPoseEstimate());

        heading = 0;

        waitForStart();


        while (opModeIsActive()) {
            localizer.update();
//            drivetrainControl.update();

//            if(gamepad1.y)
//                drive.setTargetPose(new Pose2d(-70,-100,0));
//            if(gamepad1.x)
//                drive.setTargetPose(new Pose2d(-28,-106,-45));
//            if(gamepad1.dpad_down)
//                drive.setTargetPose(new Pose2d(-70,-122,0));
//            if(gamepad1.dpad_up)
//                drive.setTargetPose(new Pose2d(-60,-122,40));
            telemetry.addData("senz1", sensorDistanceback.getDistance(DistanceUnit.MM));
            telemetry.addData("senzor2", sensorDistanceback.getDistance(DistanceUnit.MM));
            telemetry.addData("x", localizer.getPoseEstimate().getX());
            telemetry.addData("y", localizer.getPoseEstimate().getY());
            telemetry.addData("poseHeading", localizer.getPoseEstimate().getHeading());
            telemetry.addData("pos",localizer.localizer.getWheelPositions());
//            telemetry.addData("Hz", 1.0 / timer.seconds());
//            drive.telemetry(telemetry);
//            timer.reset();


            telemetry.update();
        }


    }
}
