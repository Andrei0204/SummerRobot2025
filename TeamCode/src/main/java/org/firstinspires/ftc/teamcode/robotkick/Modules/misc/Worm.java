package org.firstinspires.ftc.teamcode.robotkick.Modules.misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;



import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

@Config
public class Worm implements IStateBasedModule, IRobotModule {

    private static final double ticksDegree = 41.821;    //test
    public static final double downPos = 0, firstLevel = 3 * ticksDegree;
    public static double currentPosition = downPos;
    public static double target = 0; //cm
    public static double kP = 0.0001, kI = 0.01, kD = 0.0001; //TODO Valori PID!!!!
    private static boolean ENABLED = true;
    public final CoolMotor Worm;
    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);
    public State state;
    WormRunMode runMode;


    public Worm(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {

            Worm = null;
        } else {
            Worm = new CoolMotor(hardwareMap.get(DcMotorEx.class, "Worm"), CoolMotor.RunMode.RUN);
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

    public WormRunMode getRunMode() {
        return runMode;
    }

    public void setRunMode(WormRunMode runMode) {
        this.runMode = runMode;
    }

    public double getCurrentPosition() {
        return Worm.getCurrentPosition();
    }
    public double getRealCurrentPosition() {
        return Worm.getCurrentPosition()*ticksDegree;
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
        return Math.abs(target - getCurrentPosition()) <= tolerance * ticksDegree;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * ticksDegree);
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
            runMode = WormRunMode.MANUAL;
            setState(State.MANUAL);
            target = currentPosition * ticksDegree;
        } else {
            runMode = WormRunMode.PID;
            switch (state) {
                case FIRST:
                    setTarget(firstLevel);
                    break;
                case DOWN:
                    setTarget(downPos);
                    break;
                case MANUAL:
                    setTarget(target);
                    break;
            }
        }
    }

    @Override
    public void updateHardware() {
        switch (runMode) {
            case PID:
                Worm.setMode(CoolMotor.RunMode.PID);

                Worm.setPID(pid);
                Worm.calculatePower(Worm.getCurrentPosition(), target);
                break;
            case MANUAL: {
                Worm.setMode(CoolMotor.RunMode.RUN);
                Worm.setPower(manualPower);
                break;
            }
        }
        currentPosition = Worm.getCurrentPosition() / ticksDegree;
        Worm.update();
    }

    public enum State {
        DOWN(downPos),
        FIRST(firstLevel),
        MANUAL(currentPosition),
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

    public enum WormRunMode {
        PID, MANUAL
    }
}
