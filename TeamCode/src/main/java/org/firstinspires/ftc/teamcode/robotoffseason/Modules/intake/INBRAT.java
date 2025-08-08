package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolServo;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class INBRAT implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double downpos = 0;
    public static  double BeforCollect = 0.13;
    public static final double uppos = 0.5;
    public static final double middle = 0.32;
    public static final double transferv2 = 0.97;
    public static double auto=0.28;

    public int isprofiled=1;
    public static double profileMaxVelocity = 20, profileAcceleration = 20, profileDeceleration = 20;

    public CoolServo inbrat;
    public State BratState;

    public INBRAT(HardwareMap hardwareMap, State initialState) {
        inbrat = new CoolServo(hardwareMap.get(ServoImplEx.class, "inbrat"),false,profileMaxVelocity,profileAcceleration,profileDeceleration, initialState.position);

        this.BratState = initialState;
        isprofiled=1;
    }
    public State getState() {
        return BratState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        inbrat.isProfiled=true;
        switch (State) {

            case DOWN:
                inbrat.isProfiled=false;
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
            case TRANSFERV2:
                BratState = State.TRANSFERV2;
                break;
        }
        if(isprofiled==0)
        {
            inbrat.isProfiled=false;
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
            case TRANSFERV2:
                rotpose = transferv2;
                break;
        }
    }

    @Override
    public void updateHardware() {
        inbrat.setPosition(rotpose);
        inbrat.update();
    }

    public enum State {
        DOWN(downpos),
        MIDDLE(middle),
        SEMIDOWN(BeforCollect),
        AUTOSPECI(auto),
        TRANSFERV2(transferv2),
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

