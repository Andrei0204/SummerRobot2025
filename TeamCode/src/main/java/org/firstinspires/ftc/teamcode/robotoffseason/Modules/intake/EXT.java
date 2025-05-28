package org.firstinspires.ftc.teamcode.robotoffseason.Modules.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

@Config
public class EXT implements IStateBasedModule, IRobotModule {

    public static double TICKSCM = 12.83;    //test
    public static  double startpos=4.5*TICKSCM;
    public static  double zeropos=0*TICKSCM;
    public static final double finalpos=48*TICKSCM;
    public static final double midpos=12*TICKSCM;
    public static double autopose=0;

    public static double target = 0; //cm

    public static double currentPosition = startpos;
    public static double kP = 0.005 , kI = 0.0002, kD = 0.0005; //TODO Valori PID!!!!
    private static boolean ENABLED = true;
    public final CoolMotor EXT;
    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

    public State state;
    public EXTRUNMODE runMode;


    public EXT(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {

            EXT = null;
        } else {
            EXT = new CoolMotor(hardwareMap.get(DcMotorEx.class, "EXT"), CoolMotor.RunMode.RUN,true);

        }
        this.state = initialState;
    }


    public void setManualPower(double power) {
        this.manualPower = power;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double newTarget) {
        this.target = newTarget;
    }

    public EXTRUNMODE getRunMode() {
        return runMode;
    }

    public void setRunMode(EXTRUNMODE runMode) {
        this.runMode = runMode;
    }

    public double getCurrentPosition() {
        return EXT.getCurrentPosition();
    }

    public double getRealCurrentPosition() {
        return EXT.getCurrentPosition() / TICKSCM;
    }

    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public boolean reachedPosition(double tolerance) {
        if (state == State.MANUAL)
            return false;
        return (Math.abs(target - getCurrentPosition())) / TICKSCM <= tolerance;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * TICKSCM);
    }

    public void setAuto(double auto) {
        autopose = auto;
    }

    @Override
    public void update() {
        if (!ENABLED) return;
        updateStateValues();
        updateState();
        updateHardware();

    }

    @Override
    public void updateState() {
        if (!ENABLED) return;
        if (Math.abs(manualPower) >= 0.1) {
            runMode = EXTRUNMODE.MANUAL;
            setState(State.MANUAL);
            target = currentPosition * TICKSCM;
        } else {
            runMode = EXTRUNMODE.PID;
            switch (state) {
                case START:
                    setTarget(startpos);
                    break;
                case ZERO:
                    setTarget(zeropos);
                    break;
                case FULL:
                    setTarget(finalpos);
                    break;
                case MID:
                    setTarget(midpos);
                    break;

                case MANUAL:
                    setTarget(target);
                    break;
                case AUTO:
                    setTarget(autopose);
                    break;
            }
        }
    }

    @Override
    public void updateHardware() {
        switch (runMode) {
                    case PID:
                        EXT.setMode(CoolMotor.RunMode.PID);
                        pid = new PIDCoefficients(kP, kI, kD);

                        EXT.setPID(pid);


                        EXT.calculatePower(EXT.getCurrentPosition(), target);
                        break;
                    case MANUAL: {
                        EXT.setMode(CoolMotor.RunMode.RUN);
                        EXT.setPower(manualPower);
                        break;
                    }
                }


        currentPosition = EXT.getCurrentPosition() / TICKSCM;
        EXT.update();
    }

    public enum State {
        FULL(finalpos),
        MANUAL(currentPosition),
        AUTO(autopose),
        MID(midpos),

        ZERO(zeropos),
        START(startpos)
        ;

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

    public enum EXTRUNMODE {
        PID, MANUAL
    }
}
