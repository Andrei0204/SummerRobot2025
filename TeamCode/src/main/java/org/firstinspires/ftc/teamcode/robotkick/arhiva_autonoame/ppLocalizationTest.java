package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.MisuDrivetrainBigPP;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.PLMLocalizer;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.ppMechanumDrive;

@Config
@Disabled

@TeleOp(name = "ppLocalizationTest", group = "1Misu")
public class ppLocalizationTest extends LinearOpMode {

    PLMLocalizer Localizer;
    SampleMecanumDrive Wheels;
    ppMechanumDrive Drive;
    ElapsedTime Timer;

    MisuDrivetrainBigPP Control;

    double currentTime, lastTime, diffTime=0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PLMLocalizer(hardwareMap);
        Drive = new ppMechanumDrive(Wheels, Localizer, ppMechanumDrive.RunMode.VectorAndSlows, true);
        Control = new MisuDrivetrainBigPP(gamepad1, Drive);
        Timer = new ElapsedTime();

        waitForStart();

        while(opModeIsActive()){
            Drive.update();
            Localizer.update();
            Control.update();

            currentTime = Timer.seconds();
            diffTime = currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("X", Localizer.getCurrentPosition().getX());
            telemetry.addData("Y", Localizer.getCurrentPosition().getY());
            telemetry.addData("Heading", Localizer.getCurrentPosition().getHeading());

            telemetry.addData("Frequency", 1.0/diffTime);
            telemetry.addData("pp Frequency", Localizer.getFrequency());
            telemetry.update();
        }

    }
}
