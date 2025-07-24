package org.firstinspires.ftc.teamcode.robotoffseason.Modules;

import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;

import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.WEELIE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT2;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.PTO;

public class SaibaModules implements IRobotModule {


    public EXT EXT;
    public INCLAW INCLAW;
    public INBRAT INBRAT;
    public INROTATOR INROTATOR;
    public INTUR INTUR;
    public WEELIE WEELIE;

    public LIFT LIFT;

    public LINKEAGE LINKEAGE;
    public OUTBRAT OUTBRAT;
    public OUTCLAW OUTCLAW;
    public OUTROTATIE OUTROTATIE;
    public PTO PTO;

    private final double armLength = 15; //cm
    private double extension = 0;
    private double angle = 0;




    public SaibaModules(HardwareMap hardwareMap) {

        EXT = new EXT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT.State.START);
        LIFT = new LIFT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT.State.START);
        INCLAW = new INCLAW(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW.State.RELEASE);
        INBRAT = new INBRAT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT.State.UP);
        INROTATOR = new INROTATOR(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR.State.THIRD);
        INTUR = new INTUR(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR.State.TRANSFER);
        WEELIE = new WEELIE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.WEELIE.State.NERIDICAT);


        LINKEAGE = new LINKEAGE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE.State.CLOSED);
        OUTBRAT = new OUTBRAT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT.State.TRANSFER);
        OUTCLAW = new OUTCLAW(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW.State.RELEASE);
        OUTROTATIE = new OUTROTATIE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE.State.COLLECT);
        PTO = new PTO(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.PTO.State.FAST);


    }
    public SaibaModules(HardwareMap hardwareMap,int auto) {

        if(auto==1) {


            EXT = new EXT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT.State.ZERO);
            LIFT = new LIFT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LIFT.State.START);
            INCLAW = new INCLAW(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INCLAW.State.RELEASE);
//            INBRAT = new INBRAT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT.State.UP);
            INBRAT = new INBRAT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT.State.MIDDLE);
            INROTATOR = new INROTATOR(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INROTATOR.State.SECOND);
//            INTUR = new INTUR(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR.State.FRONT);
            INTUR = new INTUR(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR.State.DIAG);
            WEELIE = new WEELIE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.WEELIE.State.NERIDICAT);


            LINKEAGE = new LINKEAGE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.LINKEAGE.State.CLOSED);
            OUTBRAT = new OUTBRAT(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT.State.MID);
            OUTCLAW = new OUTCLAW(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTCLAW.State.SECURE);
            OUTROTATIE = new OUTROTATIE(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTROTATIE.State.COLLECT);
              PTO = new PTO(hardwareMap, org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.PTO.State.FAST);
        }

    }

    public void calculateKinematicConsts(Point point)
    {
        angle = Math.toDegrees(Math.asin(point.getY() / armLength));
        extension = point.getX() - Math.sqrt(armLength * armLength - point.getY() * point.getY());
    }

    public void setIntakeKinematic(Point point)
    {
        calculateKinematicConsts(point);
        EXT.setState(org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT.State.AUTO);
        EXT.setAuto(extension * org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.EXT.TICKSCM);
        INTUR.setState(org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INTUR.State.AUTOSPECI);
        INTUR.setangle(angle);
        INBRAT.setState(org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake.INBRAT.State.SEMIDOWN);
    }

    public double getKinematicExtension(){
        return extension;
    }

    public double getKinematixAngle()
    {
        return angle;
    }

    public double getLiftCurrent()
    {
        return Math.abs(LIFT.LIFT1.motor.motor.getCurrent(CurrentUnit.AMPS));
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("EXTPOS",EXT.getRealCurrentPosition());
        telemetry.addData("LIFTPOS",LIFT.getRealCurrentPosition());
        telemetry.addData("LIFTPOS",LIFT.getState());
        telemetry.addData("extstate",EXT.getState());
        telemetry.addData("extstate",OUTBRAT.getState());
        telemetry.addData("extstate",WEELIE.getState());
        telemetry.addData("tare", OUTROTATIE.getState());
        telemetry.addData("tare", OUTROTATIE.rotatie.getPosition());


    }


    @Override
    public void update() {
        EXT.update();
        LIFT.update();
        INCLAW.update();
        INBRAT.update();
        INROTATOR.update();
        INTUR.update();
        WEELIE.update();
        PTO.update();
        LINKEAGE.update();
        OUTBRAT.update();
        OUTROTATIE.update();
        OUTCLAW.update();
    }
}
