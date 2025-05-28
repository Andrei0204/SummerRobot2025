package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;


@TeleOp
public class MotorTest extends LinearOpMode {



    DcMotor g1;
    DcMotor g2;

    @Override
    public void runOpMode() throws InterruptedException {
        g1 =hardwareMap.get(DcMotor.class,"GL");
        g2 =hardwareMap.get(DcMotor.class,"GR");

        g1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        g1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        g2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        g2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        g1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) { // singurul while
            if(Math.abs(gamepad1.left_stick_y )>0.05){
                g1.setPower(-gamepad1.left_stick_y );
            }
            else
                g1.setPower(0);
            if(Math.abs(gamepad1.right_stick_y )>0.05){
                g2.setPower(-gamepad1.right_stick_y );

            }
            else
                g2.setPower(0);




            telemetry.addData("powerg1",g1.getPower());
            telemetry.addData("powerg2",g2.getPower());

            telemetry.update();
        }

    }
}


