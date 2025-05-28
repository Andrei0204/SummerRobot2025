package org.firstinspires.ftc.teamcode.roboDrive.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roboDrive.Auto2.AnotherMecanumDrivePedroPP;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.ppMechanumDrive;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;

public class MisuDrivetrainBigPedroPP implements IRobotModule {

    private final Gamepad gamepad;
    private final StickyGamepad stickyGamepad;
    private final AnotherMecanumDrivePedroPP drive;
    public Lift lift;
    public Pivot pivot;
    boolean haslift = false;
    private double left_stick_x = 0, left_stick_y = 0, right_stick_x = 0;
    private double slowForward, slowBack, slowTurn;



    public MisuDrivetrainBigPedroPP(Gamepad gamepad, AnotherMecanumDrivePedroPP drive) {
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;
        this.lift = null;
        haslift = false;

    }

    @Override
    public void update() throws InterruptedException {

            drive.setTargetVectorAndSlows
                    (
                            new Vector(-gamepad.left_stick_x, +gamepad.left_stick_y, gamepad.right_stick_x),
                            gamepad.right_trigger - gamepad.left_trigger,
                            gamepad.left_bumper, gamepad.right_bumper
                    );



        stickyGamepad.update();
    }
}