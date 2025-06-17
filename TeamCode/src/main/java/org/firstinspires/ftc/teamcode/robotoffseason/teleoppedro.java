package org.firstinspires.ftc.teamcode.robotoffseason;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.drive.SaibaControl;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "1Saiba", group = "Examples")
public class teleoppedro extends OpMode {

    ElapsedTime Timer;
    SaibaModules modules;
    SaibaControl modulesControl;


    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap,FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        modules = new SaibaModules(hardwareMap);
        modulesControl = new SaibaControl(gamepad1, gamepad2, modules);
        Timer = new ElapsedTime();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
        if(gamepad1.a) {
            modules.EXT.EXT.resetEncoder();
            modules.LIFT.LIFT1.resetEncoder();
            modules.LIFT.LIFT2.resetEncoder();

        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        modules.update();
        modulesControl.update();
        if(modulesControl.stickyGamepad2.a)
            telemetry.addData("idk anymore",modulesControl.clawintake);

        modules.telemetry(telemetry);
        telemetry.update();



        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}