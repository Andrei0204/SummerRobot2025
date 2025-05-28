package org.firstinspires.ftc.teamcode.newEminescu.modules.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class Prastie implements IRobotModule, IStateBasedModule {


    public Servo PrastieServo;
    public double position;
    public static double securePos = 0, launchPos = 1;

    public Prastie(HardwareMap hardwareMap, State initialState) {
        PrastieServo = hardwareMap.get(Servo.class, "Prastie"); //5
        setState(initialState);

    }

    public enum State {
        SECURE(securePos), LAUNCH(launchPos),
        ;
        double position;

        State(double position) {
            this.position = position;
        }
    }

    public State state;

    public void setState(State state) {
        if (state == this.state) return;
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    @Override
    public void update() {
        updateHardware();
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateHardware() {
        PrastieServo.setPosition(state.position);
    }
}
