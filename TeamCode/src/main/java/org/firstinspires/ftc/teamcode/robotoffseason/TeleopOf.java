package org.firstinspires.ftc.teamcode.robotoffseason;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.roboDrive.gamepads.MisuDrivetrainBigPedroPP;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.drive.SaibaControl;

@Config
@TeleOp(name = "1Saiba")
public class TeleopOf extends LinearOpMode {

    SampleMecanumDrive Wheels;
    AnotherMecanumDrivePedroPP Drive;
    ElapsedTime Timer;
    SaibaModules modules;
    SaibaControl modulesControl;
    MisuDrivetrainBigPedroPP Control;

    double currentTime, lastTime, diffTime=0;

    @Override
    public void runOpMode() throws InterruptedException
    {
         modules = new SaibaModules(hardwareMap);
        Wheels = new SampleMecanumDrive(hardwareMap);
        Drive = new AnotherMecanumDrivePedroPP(Wheels,  AnotherMecanumDrivePedroPP.RunMode.VectorAndSlows, true);
        Control = new MisuDrivetrainBigPedroPP(gamepad1, Drive);
        modulesControl = new SaibaControl(gamepad1, gamepad2, modules);
        Timer = new ElapsedTime();

        while(opModeInInit())
        {
            if(gamepad1.a) {
                modules.EXT.EXT.resetEncoder();
                modules.LIFT.LIFT1.resetEncoder();
                modules.LIFT.LIFT2.resetEncoder();

            }
        }


       // Localizer.setPose(new Pose2d(0,0,0));

        while(opModeIsActive()){
            Drive.update();
            Control.update();
            modules.update();
            modulesControl.update();
            if(modulesControl.stickyGamepad2.a)
                telemetry.addData("idk anymore",modulesControl.clawintake);

            modules.telemetry(telemetry);
            telemetry.update();

        }

    }
}
