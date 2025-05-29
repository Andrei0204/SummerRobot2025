package org.firstinspires.ftc.teamcode.roboDrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

import java.util.ArrayList;
import java.util.List;


@Config
public class SexyLocalizer {

    public static boolean ENABLED = true;
    public static double filterParameter = 0.8;
    public static double xDeceleration = 260, yDeceleration = 250; //45 cu 120
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);
 //   public RoadRunnerThreeWheelLocalizer localizer;
    public Vector glideDelta = new Vector();
    protected Pose2d pose;
    private Vector velocity = new Vector();


    public SexyLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        List<Integer> lastEncPositions = new ArrayList<>();
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        List<Integer> lastEncVels = new ArrayList<>();
        lastEncVels.add(0);
        lastEncVels.add(0);
        lastEncVels.add(0);
//localizer = new RoadRunnerThreeWheelLocalizer(hardwareMap, lastEncPositions, lastEncVels);
        pose = new Pose2d();
    }

//    public void startIMUThread(LinearOpMode opMode){
//        localizer.drive.startIMUThread(opMode);
//    }

    public void setPose(Pose2d pose) {
       // localizer.setPoseEstimate(pose);
        this.pose = pose;
    }

    public Pose2d getPoseEstimate() {
        return pose;
    }

    public Pose2d getPredictedPoseEstimate() {
        return new Pose2d(pose.getX() + glideDelta.getX(), pose.getY() + glideDelta.getY(), pose.getHeading());
    }

    public double getHeading() {
        return pose.getHeading();
    }

    public Vector getVelocity() {
        return velocity;
    }

    public void update() {
        if (!ENABLED) return;
//
//        localizer.update();
//        Pose2d pose2d = localizer.getPoseEstimate();
//        pose = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
//        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
//        Vector predictedGlideVector = new Vector(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / (2.0 * xDeceleration),
//                Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / (2.0 * yDeceleration));
//        glideDelta = Vector.rotateBy(predictedGlideVector, -pose.getHeading());
    }
}