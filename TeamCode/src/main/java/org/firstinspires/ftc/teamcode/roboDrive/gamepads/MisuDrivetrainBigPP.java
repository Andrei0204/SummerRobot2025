package org.firstinspires.ftc.teamcode.roboDrive.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roboDrive.AnotherMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;
import org.firstinspires.ftc.teamcode.robotkick.Modules.misc.ppMechanumDrive;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Lift;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Pivot;

public class MisuDrivetrainBigPP implements IRobotModule {

    private final Gamepad gamepad;
    private final StickyGamepad stickyGamepad;
    private final ppMechanumDrive drive;
    public Lift lift;
    public Pivot pivot;
    boolean haslift = false;
    private double left_stick_x = 0, left_stick_y = 0, right_stick_x = 0;

    public MisuDrivetrainBigPP(Gamepad gamepad, ppMechanumDrive drive, Lift lift, Pivot pivot) {
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;
        this.pivot= pivot;
        this.lift = lift;
        haslift = true;

    }

    public MisuDrivetrainBigPP(Gamepad gamepad, ppMechanumDrive drive) {
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;
        this.lift = null;
        haslift = false;

    }

    @Override
    public void update() {
        if (haslift) {
            left_stick_y = this.gamepad.left_stick_y;
            right_stick_x = this.gamepad.right_stick_x;
            left_stick_x = this.gamepad.left_stick_x;
            if (lift.getRealCurrentPosition() > 50) {
                drive.setTargetVectorAndSlowss
                        (
                                new Vector(left_stick_x, -left_stick_y, right_stick_x)
                        );
            } else {
                drive.setTargetVectorAndSlows
                        (
                                new Vector(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x),
                                gamepad.right_trigger - gamepad.left_trigger,
                                false, false
                        );
//                drive.setTargetVectorAndSlows
//                        (
//                                new Vector(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x),
//                                gamepad.right_trigger - gamepad.left_trigger,
//                                gamepad.left_bumper, gamepad.right_bumper
//                        );
            }
        } else {
            drive.setTargetVectorAndSlows
                    (
                            new Vector(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x),
                            gamepad.right_trigger - gamepad.left_trigger,
                            gamepad.left_bumper, gamepad.right_bumper
                    );
        }

        stickyGamepad.update();
    }
}