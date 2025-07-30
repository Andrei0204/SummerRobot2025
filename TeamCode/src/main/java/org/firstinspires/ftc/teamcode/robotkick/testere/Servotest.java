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
    ServoImplEx test2;
    ServoImplEx test3;
    ServoImplEx test4;
//    ServoImplEx test2;
    public static double pos =0.454 ;
    public static double poslink =0.454 ;
    public static double posout =0.454 ;
    public static double posinbr =0.454 ;

    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(ServoImplEx.class,"outst");
        test1 =hardwareMap.get(ServoImplEx.class,"outdr");
        test2 =hardwareMap.get(ServoImplEx.class,"linkeage");
        test3 =hardwareMap.get(ServoImplEx.class,"outrotatie");
        test4 =hardwareMap.get(ServoImplEx.class,"inbrat");
//        test1.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        test.setPwmRange(new PwmControl.PwmRange(500, 2500));
        waitForStart();
        while (opModeIsActive()) { // singurul while
                  test.setPosition(pos);
                  test1.setPosition(pos);
                  test2.setPosition(poslink);
                  test3.setPosition(posout);
                  test4 .setPosition(posinbr);
                  //test1.setPosition(pos);
        }

    }
}


