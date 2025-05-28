package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrive2;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.SexyLocalizer2;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

@Config
@TeleOp
public class limelighttest2 extends LinearOpMode {

    private Limelight3A limelight;
    SampleMecanumDrive wheels;
    AnotherMecanumDrive2 drive;
    SexyLocalizer2 localizer;
    public StickyGamepad stickyGamepad1;
    boolean pressed=false;
    public static double constx=14.5;
    public static double consty=14.5;
    public static double constxx=12.5;
    public static double constxy=9;
    public static double servotest=0.12;

    public static double grade=0;
    public static double gradevar=Math.PI/6;
    public LLResult result=null;
    Servo test,rotator;



    @Override
    public void runOpMode() throws InterruptedException {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer2(hardwareMap, wheels);
        drive = new AnotherMecanumDrive2(wheels, localizer, AnotherMecanumDrive2.RunMode.PID, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        test =hardwareMap.get(Servo.class,"PivotS");
        limelight.pipelineSwitch(1);
        limelight.start();
        Vector rotatedPose = null;
        while (opModeInInit()){
            localizer.update();
            telemetry.addData("localizer",localizer.getPoseEstimate());
            telemetry.update();
        }
        localizer.setPose(new Pose2d(0,0,localizer.getHeading()));
        while (opModeIsActive()) {
            test.setPosition(servotest);
            drive.update();
            localizer.update();
            if(gamepad1.b){
                drive.setTargetPose(new Pose2d(localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY(), localizer.getHeading()-0.1));

            }
            if(gamepad1.x){
                drive.setTargetPose(new Pose2d(localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY(), localizer.getHeading()+0.1));
            }
            if(gamepad1.a) {
                result = limelight.getLatestResult();
                if (result != null) {
                    double ypos = (Math.tan(Math.toRadians(result.getTx())) * constxy);
                  //  double ypos = (Math.tan(Math.toRadians(result.getTx())) * consty);
                    rotatedPose = new Vector(-((constx*Math.tan(gradevar+ Math.toRadians(result.getTy())))-constxx),
                            ypos,
                            0);
                    drive.setTargetPose(new Pose2d(localizer.getPoseEstimate().getX()+rotatedPose.getX() , localizer.getPoseEstimate().getY()+rotatedPose.getY(),grade  ));
                }

            }

            telemetry.addData("localizer", localizer.getPoseEstimate());
            telemetry.addData("localizer", drive.getTargetPose());
            telemetry.addData("localizer", rotatedPose);
            telemetry.update();
        }

    }
}


