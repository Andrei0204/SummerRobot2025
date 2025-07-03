package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

@Config
public class OUTBRAT2 implements IStateBasedModule, IRobotModule {
    public double rotpose;
    public static final double transferpos = 1 ;
    public static final double specimenpos = 0;
    public static final double scorespecipos = 0.68;
    public static  double samplepos = 0.34 ;
    public static final double midpos = 0.93;
    public static double auto=0.28;
    public static double profileMaxVelocity = 16, profileAcceleration = 12, profileDeceleration = 5;

    public ServoImplEx outdr  ;
    public ServoImplEx outst;
    public State OutbratState;

    public OUTBRAT2(HardwareMap hardwareMap, State initialState) {
        outdr = hardwareMap.get(ServoImplEx.class, "outdr");
        outst = hardwareMap.get(ServoImplEx.class, "outst");
//        outst.setPwmRange(new PwmControl.PwmRange(500,2500));
//        outdr.setPwmRange(new PwmControl.PwmRange(500,2500));
//
        this.OutbratState = initialState;

    }
    public State getState() {
        return OutbratState;
    }

    public void setAuto(double auto){
        this.auto =auto;

    }
    public void setState(State State) {
        switch (State) {

            case TRANSFER :
                OutbratState = State.TRANSFER ;
                break;
            case MID:
                OutbratState = State.MID;
                break;
            case AUTOSPECI:
                OutbratState = State.AUTOSPECI;
                break;
            case SPECIMEN:
                OutbratState = State.SPECIMEN;
                break;
            case SAMPLE:
                OutbratState = State.SAMPLE;
                break;
            case SCORESPECI:
                OutbratState = State.SCORESPECI;
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
        switch (OutbratState) {
            case TRANSFER :
                rotpose = transferpos;
                break;
            case MID:
                rotpose = midpos;
                break;

            case AUTOSPECI:
                rotpose =auto;
                break;
            case SPECIMEN:
                rotpose = specimenpos;
                break;
            case SAMPLE:
                rotpose = samplepos;
                break;
            case SCORESPECI:
                rotpose = scorespecipos;
                break;
        }
    }

    @Override
    public void updateHardware() {
        outdr.setPosition(OutbratState.position);

        outst.setPosition(OutbratState.position);



    }

    public enum State {
        TRANSFER (transferpos),
        SCORESPECI (scorespecipos),

        MID(OUTBRAT2.midpos),
        AUTOSPECI(auto),
        SPECIMEN(specimenpos),
        SAMPLE(samplepos);

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

