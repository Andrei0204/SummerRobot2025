package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class PTO implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double fast = 0.66;
    public static final double neutral = 0.73;
    public static final double slow = 0.98;
    public static double auto=0.28;

    public Servo pto;
    public State ptoState;

    public PTO(HardwareMap hardwareMap, State initialState) {
        pto = hardwareMap.get(Servo.class, "pto");
        this.ptoState = initialState;

    }
    public State getState() {
        return ptoState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case FAST:
                ptoState = State.FAST;
                break;
            case SLOW:
                ptoState = State.SLOW;
                break;
            case AUTOSPECI:
                ptoState = State.AUTOSPECI;
                break;
            case NEUTRAL:
                ptoState = State.NEUTRAL;
                break;
        }
    }
    @Override
    public void update() {
        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        switch (ptoState) {
            case FAST:
                rotpose = fast;
                break;
            case SLOW:
                rotpose = slow;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case NEUTRAL:
                rotpose = neutral;
                break;
        }
    }

    @Override
    public void updateHardware() {
        pto.setPosition(rotpose);
    }

    public enum State {
        FAST(PTO.fast),


        SLOW(PTO.slow),


        AUTOSPECI(auto),
        NEUTRAL(neutral);

        public final State nextState;
        public double position;


        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }
}

