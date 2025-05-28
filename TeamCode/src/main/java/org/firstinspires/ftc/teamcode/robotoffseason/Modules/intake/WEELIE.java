package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class WEELIE implements IStateBasedModule, IRobotModule {

    public double posdr;
    public double posst;
    public final double RIDICATDR = 0.4;
    public final double RIDICATST = 0.6,
            NERIDICATDR = 0.76,
            NERIDICATST = 0.23;

                //0.72 pt prindere

    public Servo Picst;
    public Servo Picdr;
    public State PICSTATE;

    public WEELIE(HardwareMap hardwareMap, State initalstate) {
        Picst = hardwareMap.get(Servo.class, "picst");
        Picdr = hardwareMap.get(Servo.class, "picdr");
        setState(initalstate);
    }

    public State getState() {
        return PICSTATE;
    }
    public void setState(State side) {
        switch (side) {

            case RIDICAT:
                PICSTATE = State.RIDICAT;
                break;
            case NERIDICAT:
                PICSTATE = State.NERIDICAT;
                break;
//            case INTERIOR:
//                PICSTATE = State.INTERIOR;
//                break;
//
//            case SEMIOPEN:
//                PICSTATE = State.SEMIOPEN;
//                break;
        }
    }

    @Override
    public void update() {
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (PICSTATE) {
            case RIDICAT:
                posst = RIDICATST;
                posdr = RIDICATDR;
                break;
            case NERIDICAT:
                posst = NERIDICATST;
                posdr = NERIDICATDR;
                break;
//            case INTERIOR:
//                clawpos = clawInterior;
//                break;
//            case SEMIOPEN:
//                clawpos = semiopenpos;
//                break;
        }
    }

    @Override
    public void updateHardware() {
        Picst.setPosition(posst);
        Picdr.setPosition(posdr);

    }

    public enum State {
        NERIDICAT,
        RIDICAT,
        ;
    }
}
