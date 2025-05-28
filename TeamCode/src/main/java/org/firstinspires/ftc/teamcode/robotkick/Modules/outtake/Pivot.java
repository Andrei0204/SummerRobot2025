package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.roboDrive.SexyLocalizer;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;
import org.opencv.core.Mat;

@Config
public class Pivot implements IStateBasedModule, IRobotModule {

    public static final double ticksDegree = 8.962;    //test
    public static final double downPos = 0 * ticksDegree;
    public static final double middlelevel = 96 * ticksDegree;
    public static final double entrylevel = 96 * ticksDegree;
    public static final double initlevel = 62 * ticksDegree;
    public static final double picklevel = 50 * ticksDegree;
    public static final double wallCollectPos = 81 * ticksDegree;
    public static double autopose = 0;

    public static double currentPosition = downPos;
    public static double target = 0; //cm
    public double max = 1; //cm
    public static double constant = -0.25;
    public static double constanttrigjos = -0.135;
    public static double constanttrigsus = 0.005;

    public static double kP = 0.00234, kI = 0, kD = 0.000213; //TODO Valori PID!!!!
    public static double kPurc = 0.0019, kIurc = 0, kDurc = 0.000165; //TODO Valori PID!!!!
    private static boolean ENABLED = true;
    public final CoolMotor Pivot;
    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

    public State state;
    public PivotRunmode runMode;
    public Lift lift;
    public SexyLocalizer sexyLocalizer = null;

    public Pivot(HardwareMap hardwareMap, Lift lift, State initialState) {
        if (!ENABLED) {

            Pivot = null;
        } else {
            this.lift = lift;
            Pivot = new CoolMotor(hardwareMap.get(DcMotorEx.class, "Pivot"), CoolMotor.RunMode.RUN);
        }
        this.state = initialState;
    }

    public Pivot(HardwareMap hardwareMap, Lift lift, State initialState, SexyLocalizer sexyLocalizera) {
        if (!ENABLED) {

            Pivot = null;
        } else {
            this.lift = lift;
            Pivot = new CoolMotor(hardwareMap.get(DcMotorEx.class, "Pivot"), CoolMotor.RunMode.RUN);
        }
        this.state = initialState;
        this.sexyLocalizer = sexyLocalizer;

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

    public PivotRunmode getRunMode() {
        return runMode;
    }

    public void setRunMode(PivotRunmode runMode) {
        this.runMode = runMode;
    }

    public double getCurrentPosition() {
        return Pivot.getCurrentPosition();
    }

    public double getRealCurrentPosition() {
        return Pivot.getCurrentPosition() / ticksDegree;
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
        return (Math.abs(target - getCurrentPosition())) / ticksDegree <= tolerance;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * ticksDegree);
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
            runMode = PivotRunmode.MANUAL;
            setState(State.MANUAL);
            target = currentPosition * ticksDegree;
        } else {
            runMode = PivotRunmode.PID;
            switch (state) {

                case DOWN:
                    setTarget(downPos);
                    break;
                case MIDDLE:
                    setTarget(middlelevel);
                    break;
                case INIT:
                    setTarget(initlevel);
                    break;
                case ENTRY:
                    setTarget(entrylevel);
                    break;
                case PICKUP:
                    setTarget(picklevel);
                    break;
                case MANUAL:
                    setTarget(target);
                    break;
                case WALLCOLLECT:
                    setTarget(wallCollectPos);
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
                Pivot.setMode(CoolMotor.RunMode.PID);
//                if(state==State.PICKUP||state==State.DOWN)
                PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);
                Pivot.setPID(pid);

                if (target == downPos) {

                    pid = new PIDCoefficients(kPurc, kIurc, kDurc);
                    Pivot.setPIDF(pid, constant);

                    Pivot.calculatePowerMax(Pivot.getCurrentPosition(), target, max);
                    Pivot.feedforward = 0;

                }
                else if (target == wallCollectPos || (target>= picklevel- 1.9*ticksDegree && target <= picklevel + 5*ticksDegree) )  {

                    pid = new PIDCoefficients(kP, kI, kD);
                    if(getCurrentPosition()>target+ 4*ticksDegree)
                     Pivot.setPIDF(pid, constanttrigjos * Math.sin(Math.toRadians(getRealCurrentPosition())));
                    else
                        Pivot.setPIDF(pid, constanttrigsus * Math.sin(Math.toRadians(getRealCurrentPosition())));
                    Pivot.calculatePowerMax(Pivot.getCurrentPosition(), target, max);
                    Pivot.feedforward = 0;

                } else if (target == middlelevel) {

                    Pivot.calculatePowerMax(Pivot.getCurrentPosition(), target, max);


                }
                else {
                        Pivot.calculatePowerMax(Pivot.getCurrentPosition(), target, max);
                }
                break;

            case MANUAL: {
                Pivot.setMode(CoolMotor.RunMode.RUN);
                Pivot.setPower(manualPower);
                if (Math.abs(manualPower) > 0.4)
                    Pivot.setPower(0.4 * Math.signum(manualPower));
                break;
            }
        }
        currentPosition = Pivot.getCurrentPosition() / ticksDegree;
        Pivot.update();
    }

    public enum State {
        DOWN(downPos),
        MIDDLE(middlelevel),
        INIT(initlevel),
        ENTRY(entrylevel),
        PICKUP(picklevel),
        MANUAL(currentPosition),
        WALLCOLLECT(wallCollectPos),
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

    public enum PivotRunmode {
        PID, MANUAL
    }
}
