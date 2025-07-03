package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class INCLAW implements IStateBasedModule, IRobotModule {

    public double clawpos;
    public final double clawRelease = 1,
            clawSecure = 0.77,
            clawInterior=0.77,
            semiopenpos=0.77;

                //0.72 pt prindere

    public Servo Claw;
    public State ClawState;

    public INCLAW(HardwareMap hardwareMap, State initalstate) {
        Claw = hardwareMap.get(Servo.class, "inclaw");
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

            case SEMIOPEN:
                ClawState = State.SEMIOPEN;
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
            case SEMIOPEN:
                clawpos = semiopenpos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        Claw.setPosition(clawpos);
    }

    public enum State {
        RELEASE,
        SEMIOPEN,
        SECURE,
        INTERIOR,
        ;
    }
}
