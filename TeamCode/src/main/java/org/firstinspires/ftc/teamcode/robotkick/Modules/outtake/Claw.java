package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class Claw implements IStateBasedModule, IRobotModule {

    public double clawpos;
    public final double clawRelease = 0.31,
            clawSecure = 0.84,
            clawInterior=0.81;

                //0.72 pt prindere

    public Servo Claw;
    public State ClawState;

    public Claw(HardwareMap hardwareMap,State initalstate) {
        Claw = hardwareMap.get(Servo.class, "Claw");
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
