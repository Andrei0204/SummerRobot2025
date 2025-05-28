package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

import java.security.SecurityPermission;

public class Rotator implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double fstpos = 0.145;
    public static final double scndpos = 0.245;
    public static final double trdpos = 0.42;
    public static final double fthpos = 0.565;
    public static final double fifthpos=0.71;
    public static final double specimenpos=1;
    public static double custom=0.47;

    public Servo rotator;
    public State RotState;

    public Rotator(HardwareMap hardwareMap, State initialState) {
        rotator = hardwareMap.get(Servo.class, "Rotator");
        this.RotState = initialState;

    }
    public State getState() {
        return RotState;
    }
    public void setCustom(double custom){
        this.custom=custom;
    }
    public void setState(State State) {
        switch (State) {

            case FIRST:
                RotState = State.FIRST;
                break;
            case SECOND:
                RotState = State.SECOND;
                break;
            case THIRD:
                RotState = State.THIRD;
                break;
            case FOURTH:
                RotState = State.FOURTH;
                break;
            case FIFTH:
                RotState = State.FIFTH;
                break;
            case SPECIMEN:
                RotState = State.SPECIMEN;
                break;
            case CUSTOM:
                RotState = State.CUSTOM;
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
        switch (RotState) {
            case FIRST:
                rotpose = fstpos;
                break;
            case SECOND:
                rotpose = scndpos;
                break;
            case THIRD:
                rotpose =trdpos;
                break;
            case FOURTH:
                rotpose =fthpos;
                break;
            case FIFTH:
                rotpose =fifthpos;
                break;
            case SPECIMEN:
                rotpose = specimenpos;
                break;
            case CUSTOM:
                rotpose = custom;
                break;
        }
    }

    @Override
    public void updateHardware() {
        rotator.setPosition(rotpose);
    }

    public enum State {
        FIRST(fstpos),
        SECOND(scndpos),
        THIRD(trdpos),
        FOURTH(fthpos),
        FIFTH(fifthpos),
        SPECIMEN(specimenpos),
        CUSTOM(custom);
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

