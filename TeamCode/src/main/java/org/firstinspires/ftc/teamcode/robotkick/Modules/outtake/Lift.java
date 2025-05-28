package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

@Config
public class Lift implements IStateBasedModule, IRobotModule {

    public static final double ticksCm = 34.33;
    public static final double downPos = 0, firstLevel = 21 * ticksCm, secondLevel = 66 * ticksCm,specimenlevel= 13  *ticksCm;
    public static double autopose=0;
    public static double currentPosition = downPos;
    public static double target = 0; //cm
    public static double kP = 0.003, kI = 0, kD = 0.00001,f=0.04; // a fost p=0.007, i=0, d = 0;
    private static boolean ENABLED = true;
    private static boolean leftReverse = true, rightReverse = false;
    public final CoolMotor glisLeft, glisRight;
    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);
    public State state;
    LiftRunMode runMode;

    public Lift(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {
            glisLeft = null;
            glisRight = null;
        } else {
            glisRight = new CoolMotor(hardwareMap.get(DcMotorEx.class, "GR"), CoolMotor.RunMode.RUN, rightReverse);
            glisLeft = new CoolMotor(hardwareMap.get(DcMotorEx.class, "GL"), CoolMotor.RunMode.RUN, leftReverse);
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

    public LiftRunMode getRunMode() {
        return runMode;
    }

    public void setRunMode(LiftRunMode runMode){
        this.runMode = runMode;
    }

    public double getCurrentPosition() {
        return glisRight.getCurrentPosition();
    }
    public double getRealCurrentPosition() {
        return glisRight.getCurrentPosition()/ticksCm;
    }

    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public boolean reachedPosition(double tolerance) {
        if (state == State.MANUAL) return false;
        return Math.abs(target - getCurrentPosition()) <= tolerance*ticksCm;
    }
    public void setAuto(double auto){
        autopose=auto;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * ticksCm);
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
            runMode = LiftRunMode.MANUAL;
            setState(State.MANUAL);
            target = currentPosition * ticksCm;
        } else {
            runMode = LiftRunMode.PID;
            switch(state){
                case FIRST:
                    setTarget(firstLevel);
                    break;
                case DOWN:
                    setTarget(downPos);
                    break;
                case MANUAL:
                    setTarget(target);
                    break;
                case SECOND:
                    setTarget(secondLevel);
                    break;
                case SPECIMEN:
                    setTarget(specimenlevel);
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
                glisRight.setMode(CoolMotor.RunMode.PID);
                glisLeft.setMode(CoolMotor.RunMode.PID);

                glisRight.setPIDF(pid,f);
                glisLeft.setPIDF(pid,f);

                glisRight.calculatePower(glisRight.getCurrentPosition(), target);
                glisLeft.calculatePower(glisRight.getCurrentPosition(), target);
                break;
            case MANUAL: {
                glisRight.setMode(CoolMotor.RunMode.RUN);
                glisLeft.setMode(CoolMotor.RunMode.RUN);

                glisRight.setPower(manualPower);
                glisLeft.setPower(manualPower);
                break;
            }
        }
        currentPosition = glisRight.getCurrentPosition() / ticksCm;
        glisLeft.update();
        glisRight.update();
    }

    public enum State {
        DOWN(downPos),
        FIRST(firstLevel),
        SECOND(secondLevel),
        SPECIMEN(specimenlevel),
        MANUAL(currentPosition),
        AUTO(autopose),
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

    public enum LiftRunMode {
        PID, MANUAL
    }
}
