package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class INBRAT implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double downpos = 0;
    public static  double BeforCollect = 0.2;
    public static final double uppos = 0.5;
    public static final double middle = 0.35;
    public static double auto=0.28;

    public Servo inbrat;
    public State BratState;

    public INBRAT(HardwareMap hardwareMap, State initialState) {
        inbrat = hardwareMap.get(Servo.class, "inbrat");
        this.BratState = initialState;

    }
    public State getState() {
        return BratState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case DOWN:
                BratState = State.DOWN;
                break;
            case SEMIDOWN:
                BratState = State.SEMIDOWN;
                break;
            case MIDDLE:
                BratState = State.MIDDLE;
                break;
            case AUTOSPECI:
                BratState = State.AUTOSPECI;
                break;
            case UP:
                BratState = State.UP;
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
        switch (BratState) {
            case DOWN:
                rotpose = downpos;
                break;
            case SEMIDOWN:
                rotpose = BeforCollect;
                break;
            case MIDDLE:
                rotpose = middle;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case UP:
                rotpose = uppos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        inbrat.setPosition(rotpose);
    }

    public enum State {
        DOWN(downpos),
        MIDDLE(middle),
        SEMIDOWN(BeforCollect),
        AUTOSPECI(auto),
        UP(uppos);

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

