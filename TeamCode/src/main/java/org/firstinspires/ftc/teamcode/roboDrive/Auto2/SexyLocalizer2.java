package org.firstinspires.ftc.teamcode.roboDrive.Auto2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

import java.util.ArrayList;
import java.util.List;


@Config
public class SexyLocalizer2 {

    public static boolean ENABLED = true;
    public static double filterParameter = 0.9;
    public static double xDeceleration = 500, yDeceleration = 450 ; //45 cu 45O Y
    //480 x pt preload
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);
    public ThreeWheelLocalizer2 localizer;
    public Vector glideDelta = new Vector();
    protected Pose2d pose;
    private Vector velocity = new Vector();
    public SampleMecanumDrive drive;

    public Boolean Useimu=false;
    public ElapsedTime imuTimer;
    LowPassFilter headingFilter;
    double imuHeading =0;
    public SexyLocalizer2(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        imuTimer = new ElapsedTime(0);
        imuTimer.reset();
        headingFilter = new LowPassFilter(0.9, 0); // -90 pt sample auto
        Useimu=false;
        List<Integer> lastEncPositions= new ArrayList<>();
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        List<Integer> lastEncVels= new ArrayList<>();
        lastEncVels.add(0);
        lastEncVels.add(0);
        lastEncVels.add(0);
        localizer = new ThreeWheelLocalizer2(hardwareMap,lastEncPositions, lastEncVels);
        pose = new Pose2d();
        this.drive=drive;
    }
    public SexyLocalizer2(HardwareMap hardwareMap, SampleMecanumDrive drive,boolean useimu) {
        imuTimer = new ElapsedTime(0);
        imuTimer.reset();
        headingFilter = new LowPassFilter(0.9, 0); // -90 pt sample auto
        Useimu=useimu;
        List<Integer> lastEncPositions= new ArrayList<>();
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        lastEncPositions.add(0);
        List<Integer> lastEncVels= new ArrayList<>();
        lastEncVels.add(0);
        lastEncVels.add(0);
        lastEncVels.add(0);
        localizer = new ThreeWheelLocalizer2(hardwareMap,lastEncPositions, lastEncVels);
        pose = new Pose2d();
        this.drive=drive;
    }


    public void startIMUThread(LinearOpMode opMode){
        drive.startIMUThread(opMode);
    }

    public void setPose(Pose2d pose) {
        localizer.setPoseEstimate(pose);
        this.pose = pose;
        headingFilter = new LowPassFilter(0.9, pose.getHeading()); // -90 pt sample auto
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

    public Vector getVelocity(){
        return velocity;
    }

    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
//        if(Useimu)
//        {   double imucopy= imuHeading;
//            imuHeading = drive.getanglethread();
//            imuHeading= Angles.normalize(imuHeading);// = imu heading;
//            if(imucopy!=imuHeading && Math.abs(imuHeading-pose2d.getHeading())>0.02) {
//                localizer.resetHeading(imuHeading);
//                pose = new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
//            }
//        }
        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
        Vector predictedGlideVector = new Vector(Math.signum(velocity.getX()) * velocity.getX() * velocity.getX() / (2.0 * xDeceleration),
                Math.signum(velocity.getY()) * velocity.getY() * velocity.getY() / (2.0 * yDeceleration));
        glideDelta = Vector.rotateBy(predictedGlideVector, -pose.getHeading());
    }
}