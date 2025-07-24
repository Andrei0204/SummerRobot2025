package org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.math.Angles;
import org.firstinspires.ftc.teamcode.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;
import org.opencv.core.Mat;

@Config
public class Follower {
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    Pose target = new Pose(0,0);
    public Pose localization =new Pose(0,0);
    CoolMotor frontLeft, frontRight, backLeft, backRight;
    boolean frontLeftMotorReversed = true, frontRightMotorReversed, backLeftMotorReversed = true, backRightMotorReversed;
    PIDController tpid = new PIDController(0,0,0), hpid = new PIDController(0,0,0);
    private double kp = 0.09, ki = 0, kd = 0.015     , kph = 1.3, kih = 0, kdh = 0.1;
    Vector power = new Vector(0,0);
    public static double velocityThreshold = 3.5 ;
    public static double lateralmultiplier = 1 ;

    double headingPower = 0;

    public Follower(HardwareMap hardwareMap,GoBildaPinpointDriver odo) {
        lateralmultiplier=1;
        this.odo=odo;
        frontLeft = new CoolMotor(hardwareMap.get(DcMotorEx.class, "FL"), CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hardwareMap.get(DcMotorEx.class, "FR"), CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hardwareMap.get(DcMotorEx.class, "BL"), CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hardwareMap.get(DcMotorEx.class, "BR"), CoolMotor.RunMode.RUN, backRightMotorReversed);
    }


    public boolean reachedTarget(double tolerance) {
        return getDistance(localization, target) <= tolerance;// && stopped();
    }

    public boolean reachedTargetAndStopped(double tolerance) {
        return reachedTarget(tolerance) && stopped();
    }

    public boolean reachedTargetOrStopped(double tolerance) {
        return reachedTarget(tolerance) || stopped();
    }

    public boolean reachedHeading(double tolerance) {
        double diff = (target.getHeading() - localization.getHeading()) % (2 * PI);
        while (diff > Math.PI) diff -= Math.PI * 2.0;
        while (diff < -Math.PI) diff += Math.PI * 2.0;
        return Math.abs(diff) <= tolerance;
    }
    public boolean stopped() {
        return (Math.abs(odo.getVelX(DistanceUnit.CM) )<= velocityThreshold &&Math.abs( odo.getVelY(DistanceUnit.CM)) <= velocityThreshold  ) && reachedTarget(4);
    }
    public boolean fullstop() {
        return (Math.abs(odo.getVelX(DistanceUnit.CM) )<= 0.2 &&Math.abs( odo.getVelY(DistanceUnit.CM)) <= 0.2  ) && reachedTarget(4);
    }
    public void setTarget(Pose target) {
        this.target = target;
    }

    public Pose getTarget() {
        return target;
    }

    public void update() {
        updatePath();
        updateMotors();

    }

    public void updatePath() {
        odo.update();
        localization = new Pose(odo.getPosition().getX(DistanceUnit.CM), odo.getPosition().getY(DistanceUnit.CM), odo.getHeading(AngleUnit.RADIANS));
        double xDiff = target.getX() - localization.getX();
        double yDiff = target.getY() - localization.getY();
        double angle = Math.atan2(yDiff, xDiff);
        double movementError = Math.hypot(xDiff, yDiff);
        tpid.setPID(kp, ki, kd);
        hpid.setPID(kph, kih, kdh);
        double translationalPower = tpid.calculate(-movementError, 0);
        power = Vector.fromAngleAndMagnitude(angle, translationalPower);
        power = Vector.rotateBy(power, localization.getHeading());
        double max = Math.max(Math.abs(power.getX()), Math.abs(power.getY()));
        if (max < 1)
            max = 1;
        power = new Vector(
                power.getX() / max,
                power.getY()*lateralmultiplier / max
        );
        double headingDiff = Angles.normalize(target.getHeading() - localization.getHeading());
        headingPower = hpid.calculate(-headingDiff);
    }

    public void updateMotors() {
        double x = power.getX(),
                y = power.getY(),
                heading = headingPower;

        frontLeft.setPower(x-y - heading);
        frontRight.setPower(x+y + heading);
        backLeft.setPower(x+y - heading);
        backRight.setPower(x-y + heading);
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }
    public double getDistance(Pose p1, Pose p2) {
        return Math.sqrt((p1.getX() - p2.getX()) * (p1.getX() - p2.getX()) + (p1.getY() - p2.getY()) * (p1.getY() - p2.getY()));
    }

    public void setKP(double kp) {
        this.kp = kp;
    }
    public void setKD(double kd)
    {
        this.kd = kd;
    }

}
