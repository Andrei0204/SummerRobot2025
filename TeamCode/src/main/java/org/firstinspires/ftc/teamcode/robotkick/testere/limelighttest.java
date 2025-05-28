package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.AnotherMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.SexyLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

@Config
@TeleOp
public class limelighttest extends LinearOpMode {

    private Limelight3A limelight;
    SampleMecanumDrive wheels;
    AnotherMecanumDrive drive;
    SexyLocalizer localizer;
    public StickyGamepad stickyGamepad1;
    boolean pressed=false;
    public static double constx=14;
    public static double consty=14;
    public static double grade=0;
    public LLResult result=null;
    Servo claw,rotator;


    @Override
    public void runOpMode() throws InterruptedException {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        wheels = new SampleMecanumDrive(hardwareMap);
        localizer = new SexyLocalizer(hardwareMap, wheels);
        drive = new AnotherMecanumDrive(wheels, localizer, AnotherMecanumDrive.RunMode.PID, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
                    double ypos = (Math.tan(Math.toRadians(result.getTx())) * consty);
                    rotatedPose = new Vector((-(Math.tan(Math.toRadians(result.getTy()))) * constx),
                            ypos,
                            0);
                    rotatedPose =rotatedPose.rotateBy(rotatedPose,-Math.toRadians(-0));
                    drive.setTargetPose(new Pose2d(localizer.getPoseEstimate().getX()+rotatedPose.getX() , localizer.getPoseEstimate().getY()+rotatedPose.getY(),grade  ));
                    drive.Lime = true;

                }

            }

            telemetry.addData("localizer", localizer.getPoseEstimate());
            telemetry.addData("localizer", drive.getTargetPose());
            telemetry.addData("localizer", rotatedPose);
            telemetry.update();
        }

    }
}


