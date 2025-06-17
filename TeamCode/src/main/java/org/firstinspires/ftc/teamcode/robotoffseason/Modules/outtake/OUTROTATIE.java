package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class OUTROTATIE implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double specimen = 0.27;
    public static final double scorespeci = 0.55;
    public static final double collect = 0.00;
    public static final double samplepos = 0;
    public static double auto=0.28;

    public Servo rotatie;
    public State RotatieState;

    public OUTROTATIE(HardwareMap hardwareMap, State initialState) {
        rotatie = hardwareMap.get(Servo.class, "outrotatie");
        this.RotatieState = initialState;

    }
    public State getState() {
        return RotatieState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case SPECIMEN:
                RotatieState = State.SPECIMEN;
                break;
            case COLLECT:
                RotatieState = State.COLLECT;
                break;
            case AUTOSPECI:
                RotatieState = State.AUTOSPECI;
                break;
            case SCORESPECI:
                RotatieState = State.SCORESPECI;
                break;
            case SCORESMAPLE:
                RotatieState = State.SCORESMAPLE;
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
        switch (RotatieState) {
            case SPECIMEN:
                rotpose = specimen;
                break;
            case COLLECT:
                rotpose = collect;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case SCORESPECI:
                rotpose = scorespeci;
                break;
            case SCORESMAPLE:
                rotpose = samplepos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        rotatie.setPosition(rotpose);
    }

    public enum State {
        SPECIMEN(specimen),
        COLLECT(OUTROTATIE.collect),
        AUTOSPECI(auto),
        SCORESPECI(scorespeci),
        SCORESMAPLE(samplepos);

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

