package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class OUTCLAW implements IStateBasedModule, IRobotModule {

    public double clawpos;
    public final double clawRelease =0.96,
            clawSecure = 0.68 ,
            clawInterior=0.58;


    public Servo Claw;
    public State ClawState;

    public OUTCLAW(HardwareMap hardwareMap, State initalstate) {
        Claw = hardwareMap.get(Servo.class, "outclaw");
        setState(initalstate);
    }

    public State getState() {
        return ClawState;
    }
    public void setState(State side) {
        switch (side) {

            case SECURE:
                ClawState = State.SECURE;
                break;
            case RELEASE:
                ClawState = State.RELEASE;
                break;
            case INTERIOR:
                ClawState = State.INTERIOR;
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
        switch (ClawState) {
            case SECURE:
                clawpos = clawSecure;
                break;
            case RELEASE:
                clawpos = clawRelease;
                break;
            case INTERIOR:
                clawpos = clawInterior;
                break;
        }
    }

    @Override
    public void updateHardware() {
        Claw.setPosition(clawpos);
    }

    public enum State {
        RELEASE,
        SECURE,
        INTERIOR,
        ;
    }
}
