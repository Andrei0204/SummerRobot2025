package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class LINKEAGE implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double open = 0.66;
    public static final double overcenter = 0.73;
    public static final double closed = 0.98;
    public static double auto=0.8;

    public Servo linkeage;
    public State LinkeageState;

    public LINKEAGE(HardwareMap hardwareMap, State initialState) {
        linkeage = hardwareMap.get(Servo.class, "linkeage");
        this.LinkeageState = initialState;

    }
    public State getState() {
        return LinkeageState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case OPEN :
                LinkeageState = State.OPEN ;
                break;
            case CLOSED:
                LinkeageState = State.CLOSED;
                break;
            case AUTOSPECI:
                LinkeageState = State.AUTOSPECI;
                break;
            case OVERCENTER:
                LinkeageState = State.OVERCENTER;
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
        switch (LinkeageState) {
            case OPEN :
                rotpose = open;
                break;
            case CLOSED:
                rotpose = closed;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case OVERCENTER:
                rotpose = overcenter;
                break;
        }
    }

    @Override
    public void updateHardware() {
        linkeage.setPosition(rotpose);
    }

    public enum State {
        OPEN (open),
        CLOSED(LINKEAGE.closed),
        AUTOSPECI(auto),
        OVERCENTER(overcenter);

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

