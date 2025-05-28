package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class Servotestrotator extends LinearOpMode {
    Servo test;
    public static double pos =0.75;
    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"Rotator");

        test.setPosition(pos);
        waitForStart();
        while (opModeIsActive()) { // singurul while
               test.setPosition(pos);
            telemetry.addData("power",test.getPosition());

            telemetry.update();
        }

    }
}


