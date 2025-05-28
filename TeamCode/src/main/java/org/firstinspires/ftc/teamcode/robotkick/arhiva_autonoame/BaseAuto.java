package org.firstinspires.ftc.teamcode.robotkick.arhiva_autonoame;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robotkick.Modules.CeresModules;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.PLMLocalizer;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.ppMechanumDrive;

@Disabled
@Autonomous(name = "BaseAuto", group = "1Misu")
public class BaseAuto extends LinearOpMode {

    PLMLocalizer Localizer;
    SampleMecanumDrive Wheels;
    ppMechanumDrive Drive;
    CeresModules Modules;
    ElapsedTime Timer;

    @Override
    public void runOpMode() throws InterruptedException
    {

        Wheels = new SampleMecanumDrive(hardwareMap);
        Localizer = new PLMLocalizer(hardwareMap);
        Drive = new ppMechanumDrive(Wheels, Localizer, ppMechanumDrive.RunMode.PID, true);
        Modules = new CeresModules(hardwareMap, false, this);

        waitForStart();

        while(opModeIsActive()){
            Drive.update();
            Localizer.update();
            Modules.update();
        }

    }

    public enum States{
        FORWARD,
        BACKWARD
    }


}
