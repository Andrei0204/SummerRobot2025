package org.firstinspires.ftc.teamcode.robotkick.Modules.outtake;


import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IStateBasedModule;

public class Outtake implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    public enum State {
        DOWN,
        FIRST,
        NOPOWER,
        MANUAL
    }

   State prevState;
   State state;


    public void setState(State newState) {
        if (state == newState) return;
        state = newState;
        switch (newState) {
            case DOWN:
                lift.setState(Lift.State.DOWN);
                pivot.setState(Pivot.State.DOWN);
                break;
            case FIRST:
                lift.setState(Lift.State.FIRST);
                break;
            case NOPOWER:

        }
    }

    public Outtake.State getState() {
        return state;
    }

    public final Lift lift;
    public final Pivot pivot;

    public Outtake(Lift lift,Pivot pivot, State initialState) {
        this.lift = lift;
        lift.setState(Lift.State.DOWN);
        this.pivot = pivot;
        pivot.setState(Pivot.State.DOWN);
        this.state = initialState;
        setState(state);
    }

    @Override
    public void atStart() {
        lift.atStart();
        pivot.atStart();
    }

    @Override
    public void initUpdate() {
        lift.initUpdate();
        pivot.initUpdate();
    }

    @Override
    public void update() {
        if (!ENABLED) return;

        updateState();
        updateHardware();
    }

    @Override
    public void updateState() {
        if (Math.abs(lift.manualPower) > 0.1) lift.setRunMode(Lift.LiftRunMode.MANUAL);
        else lift.setRunMode(Lift.LiftRunMode.PID);


    }

    @Override
    public void updateHardware() {
        lift.update();
        pivot.update();

    }
}