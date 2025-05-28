package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.MisuDrivetrainBigPedroPP;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.drive.CeresControl;

@Config
@Disabled
@TeleOp(name = "111PedroFC", group = "1Misu")
public class PedroPPLocalizationTest extends LinearOpMode {

    PinpointLocalizer Localizer;
    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    ElapsedTime Timer;
    CeresModules modules;
    CeresControl modulesControl;
    MisuDrivetrainBigPedroPP Control;

    double currentTime, lastTime, diffTime=0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        modules = new CeresModules(hardwareMap, false, this);
        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0,0,0),true);
        Drive = new AnotherMecanumDrivePedroPP(Wheels, Localizer, AnotherMecanumDrivePedroPP.RunMode.Vector, true);
        Control = new MisuDrivetrainBigPedroPP(gamepad1, Drive);
        modulesControl = new CeresControl(gamepad1, gamepad2, modules);
        Timer = new ElapsedTime();

        waitForStart();

       // Localizer.setPose(new Pose2d(0,0,0));

        while(opModeIsActive()){
            Drive.update();
            Localizer.update();
            Control.update();
            modules.update();
            modulesControl.update();
            currentTime = Timer.seconds();
            diffTime = currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("X", Localizer.getPose().getX());
            telemetry.addData("Y", Localizer.getPose().getY());

            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("Heading", Localizer.getCurrentPosition().getHeading());

            telemetry.addData("Frequency", 1.0/diffTime);
//            telemetry.addData("pp Frequency", Localizer.getFrequency());
            telemetry.update();
        }

    }
}
