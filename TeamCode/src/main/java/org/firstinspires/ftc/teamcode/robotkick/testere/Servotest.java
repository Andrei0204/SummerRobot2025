package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class Servotest extends LinearOpMode {
    ServoImplEx test;
    ServoImplEx test1;
    public static double pos =0.66;
    public static double pos2 =0;
    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(ServoImplEx.class,"inclaw");
 //       test1 =hardwareMap.get(ServoImplEx.class,"outst");
//        test1.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        test.setPwmRange(new PwmControl.PwmRange(500, 2500));
        waitForStart();
        while (opModeIsActive()) { // singurul while
                  test.setPosition(pos);
                  //test1.setPosition(pos);
        }

    }
}


