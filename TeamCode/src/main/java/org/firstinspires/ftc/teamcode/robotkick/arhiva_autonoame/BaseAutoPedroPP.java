package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathingold.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;

@Config
@Disabled
@Autonomous(name = "BaseAutoPedroPP", group = "1Misu")
public class BaseAutoPedroPP extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    CeresModules Modules;
    ElapsedTime Timer;

    States currentState = States.FORWARD;

    double heading = 0;
    public static double sum = 0;
    public static int cycle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.PID, true);
//        Modules = new CeresModules(hardwareMap, false, this);

        waitForStart();

        while (opModeIsActive()) {
            sum += Localizer.getVelocityVector().getMagnitude();
            cycle++;
            switch (currentState) {
                case FORWARD:
                    Drive.setTargetPose(new Pose2d(0, 0, heading));
                    break;
                case BACKWARD:
                    Drive.setTargetPose(new Pose2d(50, 50, heading));
                    break;
            }

            if (gamepad1.a) {
                currentState = States.FORWARD;
                sum = 0;
                cycle = 0;

            }
            if (gamepad1.b) {
                currentState = States.BACKWARD;
                sum = 0;
                cycle = 0;
            }
            if (gamepad1.y) {
                heading = 90 - heading;
                while (gamepad1.y && opModeIsActive()) ;
            }

            Drive.update();
            Localizer.update();
//            Modules.update();

            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("Heading", Localizer.getPose().getHeading());
            telemetry.addData("Medie", sum / cycle);

//            telemetry.addData("Frequency", 1.0/diffTime);
//            telemetry.addData("pp Frequency", Localizer.getFrequency());
            telemetry.update();
        }

    }

    public enum States {
        FORWARD,
        BACKWARD
    }


}
