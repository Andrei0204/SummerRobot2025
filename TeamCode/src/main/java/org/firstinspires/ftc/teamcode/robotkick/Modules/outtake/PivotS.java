package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class PivotS implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double downpos = 0.01;
    public static final double wallpos = 0.27;
    public static final double diagpos = 0.35;
    public static final double scorepos=0.45;
    public static final double specipos=0.42;
    public static double auto=0.28;

    public Servo PivotS;
    public State PivState;

    public PivotS(HardwareMap hardwareMap,State initialState) {
        PivotS = hardwareMap.get(Servo.class, "PivotS");
        this.PivState = initialState;

    }
    public State getState() {
        return PivState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case DOWN:
                PivState = State.DOWN;
                break;
            case DIAGONAL:
                PivState = State.DIAGONAL;
                break;
            case SCORE:
                PivState = State.SCORE;
                break;
            case SPECIMEN:
                PivState = State.SPECIMEN;
                break;
            case AUTOSPECI:
                PivState = State.AUTOSPECI;
                break;
            case WALL:
                PivState = State.WALL;
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
        switch (PivState) {
            case DOWN:
                rotpose = downpos;
                break;
            case DIAGONAL:
                rotpose = diagpos;
                break;
            case SCORE:
                rotpose =scorepos;
                break;
            case SPECIMEN:
                rotpose =specipos;
                break;
            case AUTOSPECI:
                rotpose =auto;
                break;
            case WALL:
                rotpose =wallpos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        PivotS.setPosition(rotpose);
    }

    public enum State {
        DOWN(downpos),
        DIAGONAL(diagpos),
        SCORE(scorepos),
        AUTOSPECI(auto),
        WALL(wallpos),
        SPECIMEN(specipos);

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

