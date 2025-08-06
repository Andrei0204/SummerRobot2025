package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;
import com.acmerobotics.dashboard.config.Config;

@Config
public class OUTROTATIE implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static  double specimen = 0.41   ;
    public static  double scorespeci = 0.85;
    public static final double scorespecitele = 0.76;
    public static  double collect = 0.3;
    public static  double samplepos = 0.3;
    public static final double transfernou = 0.85;
    public static double auto=0.15;

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
            case SCORESPECITELE:
                RotatieState = State.SCORESPECITELE;
                break;
            case SCORESMAPLE:
                RotatieState = State.SCORESMAPLE;
                break;
            case TRANSFERNOU:
                RotatieState = State.TRANSFERNOU;
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
            case SCORESPECITELE:
                rotpose = scorespecitele;
                break;
            case SCORESMAPLE:
                rotpose = samplepos;
                break;
            case TRANSFERNOU:
                rotpose = transfernou;
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
        TRANSFERNOU(transfernou),
        SCORESPECITELE(scorespecitele),
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

