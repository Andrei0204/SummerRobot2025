package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class INTUR implements IStateBasedModule, IRobotModule {
    public double rotpose;
        public static final double transfer = 0.03;
    public static final double diagonal = 0.37;
    public static final double throwpos = 0.6;
    public static final double FRONT = 0.71;
    public static double auto=0.69;

    public Servo intur;
    public State InturState;

    public INTUR(HardwareMap hardwareMap, State initialState) {
        intur = hardwareMap.get(Servo.class, "intur");
        this.InturState = initialState;

    }
    public State getState() {
        return InturState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setangle(double angle)
    {
        double rightmax=0.47;
        double leftmax=0.94;
        double middle=0.71;
        double tickstodegree=0.0036;
        this.auto =middle +angle*tickstodegree;
        if(auto>leftmax)
            auto=leftmax;
        if(auto<rightmax)
            auto=rightmax;

    }
    public void setState(State State) {
        switch (State) {

            case TRANSFER:
                InturState = State.TRANSFER;
                break;
            case FRONT:
                InturState = State.FRONT;
                break;
            case AUTOSPECI:
                InturState = State.AUTOSPECI;
                break;
            case DIAG:
                InturState = State.DIAG;
                break;
            case THROW:

                InturState = State.THROW;
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
        switch (InturState) {
            case TRANSFER:
                rotpose = transfer;
                break;
            case FRONT:
                rotpose = FRONT;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case DIAG:
                rotpose = diagonal;
                break;
            case THROW:
                rotpose = throwpos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        intur.setPosition(rotpose);
    }

    public enum State {
        TRANSFER(transfer),
        FRONT(INTUR.FRONT),
        AUTOSPECI(auto),
        THROW(throwpos),
        DIAG(diagonal);

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

