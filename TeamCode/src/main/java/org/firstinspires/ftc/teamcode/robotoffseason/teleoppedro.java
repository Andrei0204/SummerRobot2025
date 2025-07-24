package org.firstinspires.ftc.teamcode.robotoffseason;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.SaibaModules;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.drive.SaibaControl;
import org.firstinspires.ftc.teamcode.robotoffseason.Modules.outtake.OUTBRAT;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
@Config
@TeleOp(name = "1Saiba")
public class teleoppedro extends OpMode {
    StickyGamepad stickyGamepad1=null;
    ElapsedTime Timer;
    SaibaModules modules;
    SaibaControl modulesControl;
    double headingCorrection;
    public  static double kp=
            -0.6
            ,ki=-0.1,kd=0.005;
    double holdheading=0;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    PIDFController headingPIDController;
    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        modules = new SaibaModules(hardwareMap);
        modulesControl = new SaibaControl(gamepad1, gamepad2, modules);
        Timer = new ElapsedTime();
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            modules.EXT.EXT.resetEncoder();
            modules.LIFT.LIFT1.resetEncoder();
            modules.LIFT.LIFT2.resetEncoder();
            follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x,  -gamepad1.right_stick_x);
            follower.update();

        }
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        modules.OUTBRAT.setState(OUTBRAT.State.MID);
        follower.startTeleopDrive();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        headingPIDController=new PIDFController(kp,ki,kd,0);
       // follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        modules.telemetry(telemetry);
        follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x,  -gamepad1.right_stick_x);
        follower.update();
        modules.update();
        stickyGamepad1.update();
        modulesControl.update();

        telemetry.update();


    }

    /**
     * We do not use this because everything automatically should disable
     **/
    @Override
    public void stop() {
    }
}