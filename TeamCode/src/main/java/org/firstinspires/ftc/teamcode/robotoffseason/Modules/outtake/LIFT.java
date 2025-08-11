package org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

@Config
public class LIFT implements IStateBasedModule, IRobotModule {

    public static double TICKSCM = 724.35;    //test
        public static double startpos =  0*TICKSCM;
    public static final double finalpos = 63 * TICKSCM;
    public static final double finalposauto = 66 * TICKSCM;
    public static final double transferpos = 0 * TICKSCM;
    public static double scorespecipos = 28.5 * TICKSCM;
    public static double scorespecipostele = 31.5 * TICKSCM;
//    public static double scorespecipos = 34 * TICKSCM;
    public static final double lowbasket = 16 * TICKSCM;
    public static double autopose = 0;

    public static double target = 0; //cm

    public static double currentPosition = startpos;
    public  double kP = 0.00017,kp2=0.0002, kI = 0.17, kD = 0.0000008, f = -0; //TODO Valori PID!!!!
    public  double kPjos = 0.000095 , kIjos = 0.1, kDjos = 0.000003; //TODO Valori PID!!!!

    private static boolean ENABLED = true;
    public CoolMotor LIFT1 = null;
    public CoolMotor LIFT2 = null;

    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

    public State state;
    public EXTRUNMODE runMode;

    VoltageSensor voltage = null;

    public LIFT(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {

            LIFT1 = null;
            LIFT2 = null;
        } else {
            LIFT1 = new CoolMotor(hardwareMap.get(DcMotorEx.class, "LIFT1"), CoolMotor.RunMode.RUN);
            LIFT2 = new CoolMotor(hardwareMap.get(DcMotorEx.class, "LIFT2"), CoolMotor.RunMode.RUN);
            voltage = hardwareMap.voltageSensor.iterator().next();

        }
        kI=0.17;
        kIjos=0.1;
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
        return LIFT1.getCurrentPosition();
    }

    public double getRealCurrentPosition() {
        return LIFT1.getCurrentPosition() / TICKSCM;
    }

    public boolean atTarget()
    {
        return Math.abs(getRealCurrentPosition() - target/TICKSCM) < 3;
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
                case FULL:
                    setTarget(finalpos);
                    break;
                case FULLAUTO:
                    setTarget(finalposauto);
                    break;
                case TRANSFER:
                    setTarget(transferpos);
                    break;
                case SCORESPECI:
                    setTarget(scorespecipos);
                    break;
                case SCORESPECItele:
                    setTarget(scorespecipostele);
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
        pid = new PIDCoefficients(kP, kI, kD);
        double actualKs = f * 12.0 / voltage.getVoltage();
        if (
                (target== transferpos ) ||
                (target == startpos)) {
            pid = new PIDCoefficients(kPjos, kIjos, kDjos);
            actualKs = 0;
        }
        if (
                (target== finalpos )) {
            pid = new PIDCoefficients(kp2, kI, kD);
            actualKs = 0;
        }
        switch (runMode) {
            case PID:
                LIFT1.setMode(CoolMotor.RunMode.PID);
                LIFT2.setMode(CoolMotor.RunMode.PID);

                LIFT1.setPIDF(pid, actualKs);
                LIFT2.setPIDF(pid, actualKs);


                LIFT2.calculatePowerFF(LIFT1.getCurrentPosition(), target);
                LIFT1.calculatePowerFF(LIFT1.getCurrentPosition(), target);
                break;
            case MANUAL: {
                LIFT2.setMode(CoolMotor.RunMode.RUN);
                LIFT1.setMode(CoolMotor.RunMode.RUN);
                LIFT2.setPower(manualPower);
                LIFT1.setPower(manualPower);
                break;
            }
        }


        currentPosition = LIFT1.getCurrentPosition() / TICKSCM;
        LIFT1.update();
        LIFT2.update();
    }

    public enum State {
        FULL(finalpos),
        MANUAL(currentPosition),
        AUTO(autopose),
        TRANSFER(transferpos),
        SCORESPECI(scorespecipos),
        SCORESPECItele(scorespecipostele),
        FULLAUTO(finalposauto),


        START(startpos);

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
