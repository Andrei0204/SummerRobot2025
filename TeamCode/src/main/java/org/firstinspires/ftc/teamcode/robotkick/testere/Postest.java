package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp
public class Postest extends LinearOpMode {



    private static final double ticksCm = 34.33;
    private static final double degreesPivot = 8.962;
    private static final double degreesWorm = 41.821;

    DcMotor Pivot;
    DcMotor GL;
    DcMotor GR;
    DcMotor worm;

    public DistanceSensor sensorDistanceback;
    public DistanceSensor sensorDistancefront;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorDistanceback = hardwareMap.get(DistanceSensor.class, "DS");
        sensorDistancefront = hardwareMap.get(DistanceSensor.class, "DSfront");
        Pivot = hardwareMap.get(DcMotor.class, "Pivot");
        GL = hardwareMap.get(DcMotor.class, "GL");
        GR = hardwareMap.get(DcMotor.class, "GR");
        worm = hardwareMap.get(DcMotor.class, "Worm");
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        worm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("posPiv", Pivot.getCurrentPosition() / degreesPivot);
            telemetry.addData("WormPOS", worm.getCurrentPosition() / degreesWorm);
            telemetry.addData("GL", GL.getCurrentPosition() / ticksCm);
            telemetry.addData("GR", GR.getCurrentPosition() / ticksCm);
            telemetry.update();


        }
    }
}


