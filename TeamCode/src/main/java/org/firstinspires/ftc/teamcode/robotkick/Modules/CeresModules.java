package org.firstinspires.ftc.teamcode.robotkick.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.Senzor_Distanta;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.Worm;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.PivotS;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Rotator;

public class CeresModules implements IRobotModule {

    public Claw claw;
    public Rotator rotator;
    public Pivot pivot;
    public Lift lift;
    public PivotS pivotS;
    public Worm worm;
    public Senzor_Distanta senzor;




    public CeresModules(HardwareMap hardwareMap, boolean auto, LinearOpMode opMode) {
        claw = new Claw(hardwareMap, Claw.State.RELEASE);
        rotator = new Rotator(hardwareMap, Rotator.State.THIRD);
        pivotS = new PivotS(hardwareMap, PivotS.State.DOWN);
        lift = new Lift(hardwareMap, Lift.State.DOWN);
        pivot = new Pivot(hardwareMap, lift, Pivot.State.INIT);
        senzor = new Senzor_Distanta(hardwareMap);
        worm = new Worm(hardwareMap, Worm.State.DOWN);

    }

    public CeresModules(HardwareMap hardwareMap, boolean auto, OpMode testpedro2) {
//        claw = new Claw(hardwareMap, Claw.State.RELEASE);
//        rotator = new Rotator(hardwareMap, Rotator.State.THIRD);
//        pivotS = new PivotS(hardwareMap, PivotS.State.DOWN);
//        lift = new Lift(hardwareMap, Lift.State.DOWN);
//        pivot = new Pivot(hardwareMap, lift, Pivot.State.INIT);
//        senzor = new Senzor_Distanta(hardwareMap);
//        worm = new Worm(hardwareMap, Worm.State.DOWN);


    }

    public void telemetry(Telemetry telemetry) {

    }


    @Override
    public void update() {
    }
}
