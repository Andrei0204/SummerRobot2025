package org.firstinspires.ftc.teamcode.newEminescu.modules.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizerImuHeading;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

@Config
public class SexyLocalizerV2 implements IRobotModule {

    public static boolean ENABLED = true;

    protected Pose2d pose;
    public TwoWheelTrackingLocalizerImuHeading localizer;

    public SexyLocalizerV2(HardwareMap hardwareMap) {
        localizer = new TwoWheelTrackingLocalizerImuHeading(hardwareMap);
        pose = new Pose2d();
    }

    public void startIMUThread(LinearOpMode opMode, SexyLocalizerV2 loc){
        this.localizer.startIMUThread(loc ,opMode);
    }

    public void setPose(Pose2d pose) {
        localizer.setPoseEstimate(pose);
        this.pose = pose;
    }

    public Pose2d getPoseEstimate() {
        return pose;
    }

    public Pose2d getPredictedPoseEstimate(){
        return new Pose2d(pose.getX() + glideDelta.getX(), pose.getY() + glideDelta.getY(), pose.getHeading());
    }

    public double getHeading(){
        return pose.getHeading();
    }

    private Vector velocity = new Vector();
    public Vector glideDelta = new Vector();

    public static double filterParameter = 0.9;
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double xDeceleration = 80, yDeceleration = 80; //45 cu 120

    public Vector getVelocity(){
        return velocity;
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose2d(pose2d.getX(), pose2d.getY(), localizer.getHeading());
        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
        Vector predictedGlideVector = new Vector(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / (2.0 * xDeceleration),
                Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / (2.0 * yDeceleration));
        glideDelta = Vector.rotateBy(predictedGlideVector, -pose.getHeading());
    }
}