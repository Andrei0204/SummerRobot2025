package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class  ServotestClaw extends LinearOpMode {
    Servo test;
    Servo test1;
    public static double pos =0.78;
    public static double pos2 =0.28;
    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"picdr");
        test1 =hardwareMap.get(Servo.class,"picst");

        test.setPosition(pos);
        test1.setPosition(pos2);
        waitForStart();
        while (opModeIsActive()) { // singurul while
           test.setPosition(pos);
           test1.setPosition(pos2);
            telemetry.addData("power",test.getPosition());

            telemetry.update();
        }

    }
}


