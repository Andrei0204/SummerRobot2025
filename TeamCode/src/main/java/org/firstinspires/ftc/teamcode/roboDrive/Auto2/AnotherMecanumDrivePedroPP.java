package org.firstinspires.ftc.teamcode.roboDrive.Auto2;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.math.Angles;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;


@Config
public class AnotherMecanumDrivePedroPP implements IRobotModule {

    public static boolean ENABLED = true;

    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = false, backLeftMotorReversed = false, backRightMotorReversed = false;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.05, 0.015, 0.0075),
            headingPID = new PIDCoefficients(
                    0.72,
                    0,
                    0.06);


    public static double lateralMultiplier = 0.65; //2.13
    public final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public final PIDController tpid = new PIDController(0, 0, 0), hpid = new PIDController(0, 0, 0);
    public static double velocityThreshold = 1.5 ;
    public static double angVelThreshold = 0.1 ;

    double targetHeading;
    public static double kPHeadinTeleop= -0.002;
    double error = 0;


    public Vector powerVector = new Vector();
    public Vector targetVector = new Vector();
    public double diff;
    private PinpointLocalizer localizer;
    private RunMode runMode;
    private Pose2d targetPose = new Pose2d();
    public static double maxpower = 0;
    Pose2d currentPose;
    public static double  headingMultiplier = 1.5;
    public static double ks = 0.03;
    public static double overallMultiplier = 1;
    public VoltageSensor voltageSensor;
    public static double voltage;

    private static double maxPowerTrans = 1;

    private double heading=0;
    public AnotherMecanumDrivePedroPP(SampleMecanumDrive drive, PinpointLocalizer localizer, RunMode runMode, boolean brake) {
        this.runMode = runMode;
        if (!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            return;
        }
        translationalPID = new PIDCoefficients(0.05, 0.015, 0.0075);
        maxPowerTrans = 1;
        this.localizer = localizer;
        headingMultiplier = 1.5;
        lateralMultiplier = 0.65; //2.13
        frontLeft = new CoolMotor(drive.leftFront, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(drive.rightFront, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(drive.leftRear, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(drive.rightRear, CoolMotor.RunMode.RUN, backRightMotorReversed);
        voltageSensor = SampleMecanumDrive.batteryVoltageSensor;

        if (brake) {
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.runMode = runMode;
    }
    public AnotherMecanumDrivePedroPP(SampleMecanumDrive drive, RunMode runMode, boolean brake) {
        this.runMode = runMode;
        if (!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            return;
        }
        headingMultiplier = 1.5;
        lateralMultiplier = 0.65; //2.13
        this.localizer = null;
        translationalPID = new PIDCoefficients(0.05, 0.015, 0.0075);
        frontLeft = new CoolMotor(drive.leftFront, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(drive.rightFront, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(drive.leftRear, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(drive.rightRear, CoolMotor.RunMode.RUN, backRightMotorReversed);
        voltageSensor = SampleMecanumDrive.batteryVoltageSensor;

        if (brake) {
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.runMode = runMode;
    }

    public void resetIMU() throws InterruptedException {
        localizer.resetPosAndIMU();
    }

    public void setTargetVector(Vector Vector) {
        this.targetVector = Vector;
    }

    public void setTargetVectorAndSlows(Vector vector, double slowTurn, boolean slowBack, boolean slowForward) {

        double yComp = 0, xComp = 0, rotate = 0;

        if (slowBack) yComp = -0.2;
        if (slowForward) yComp = 0.2;

        if(Math.abs(slowTurn)>0.08){
            rotate = slowTurn*0.4;
        }
        else rotate = vector.getZ();

        this.targetVector = new Vector(xComp != 0 ? xComp : vector.getX() * 1.1, yComp != 0 ? yComp : vector.getY(), rotate);
    }



    public void setTargetVectorAndSlowss(Vector vector) {

        double yComp = 0, xComp = 0, zComp = 0;
        yComp = vector.getY() < 0 ? Math.max(vector.getY(), -0.25) : Math.min(vector.getY(), 0.25);
        xComp = vector.getX() < 0 ? Math.max(vector.getX(), -0.25) : Math.min(vector.getX(), 0.25);
        zComp = vector.getZ() < 0 ? Math.max(vector.getZ(), -0.25) : Math.min(vector.getZ(), 0.25);
        this.targetVector = new Vector(xComp, yComp, zComp);

    }

    public RunMode getRunMode() {
        return runMode;
    }

    //200h spend on this drive

    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
    }
    public PinpointLocalizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(PinpointLocalizer localizer) {
        this.localizer = localizer;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d pose) {
        this.targetPose = new Pose2d(pose.getX(), pose.getY(), Math.toRadians(pose.getHeading()));
    }

    public boolean reachedTarget(double tolerance) {
        if (runMode == RunMode.Vector) return false;
        return getDistance(localizer.getCurrentPosition(), targetPose) <= tolerance;// && stopped();
    }

    public boolean reachedTargetAndStopped(double tolerance) {
        return reachedTarget(tolerance) && stopped();
    }

    public boolean reachedTargetOrStopped(double tolerance) {
        return reachedTarget(tolerance) || (stopped() && reachedTarget(4));
    }

    public boolean reachedHeading(double tolerance) {
        if (runMode == RunMode.Vector) return false;
        diff = (targetPose.getHeading() - localizer.getHeading()) % (2 * PI);
        while (diff > Math.PI) diff -= Math.PI * 2.0;
        while (diff < -Math.PI) diff += Math.PI * 2.0;
        return Math.abs(diff) <= tolerance;
    }

    public boolean reachedHeadingAndStopped(double tolerance) {
        return reachedHeading(tolerance) && Math.abs(localizer.getAngularVelocity())<0.1;
    }

    public boolean stopped() {
        return localizer.getVelocityVector().getMagnitude() <= velocityThreshold;
    }

    public void setMaxPower(double value){
        maxPowerTrans = value;
    }

    private void updatePowerVector() {
        switch (runMode) {
            case VectorAndSlows:
                powerVector = targetVector;
                break;
            case Vector:
                powerVector = new Vector(-targetVector.getX(), -targetVector.getY(), targetVector.getZ());
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PID:
//                currentPose = localizer.getPredictedPoseEstimate();

                currentPose = localizer.getCurrentPosition();

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                double calculatedCos = xDiff/distance;
                double calculatedSin = yDiff/distance;

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                double translationalPower = tpid.calculate(-distance, 0);

                if(maxPowerTrans<1 && Math.abs(translationalPower)>maxPowerTrans){
                    translationalPower = maxPowerTrans * Math.signum(translationalPower);
                }

                powerVector = new Vector(translationalPower * calculatedCos, translationalPower * calculatedSin);
                powerVector = Vector.rotateBy(powerVector, currentPose.getHeading());

                double headingDiff = Angles.normalize(targetPose.getHeading() - currentPose.getHeading());

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(-headingDiff, 0) * headingMultiplier;

                powerVector= new Vector(powerVector.getX(),powerVector.getY() * lateralMultiplier, headingPower);
                break;

        }
//        if(Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
//        if (Math.abs(powerVector.getMagnitudeXY()) + Math.abs(powerVector.getZ()) > 1)
//            powerVector.scaleToMagnitude(1);

        if (Math.abs(powerVector.getMagnitude()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1.2);
        powerVector.scaleBy(overallMultiplier);

    }

    private void updateMotors() {
        switch (runMode) {
            case VectorAndSlows:
            case Vector:

                double x = powerVector.getX(),
                        y = powerVector.getY(),
                        rot = powerVector.getZ();
                double frontLeftPower = y+x+rot;
                double frontRightPower = y-x-rot;
                double backLeftPower = y-x+rot;
                double backRightPower = y+x-rot;


                frontLeft.setPower(frontLeftPower);
                frontRight.setPower(frontRightPower);
                backLeft.setPower(backLeftPower);
                backRight.setPower(backRightPower);
                break;
            case PID:
                double actualKs = ks * 12.0 / voltage;

                frontLeft.setPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
                frontRight.setPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
                backLeft.setPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));
                backRight.setPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
                break;
        }

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public void update() {
        if (!ENABLED) return;
        updatePowerVector();
        updateMotors();
        voltage = voltageSensor.getVoltage();
    }

    public void telemetry(Telemetry telemetry) {

//        telemetry.addData("Atend", isAtend());
        telemetry.addData("maxpower", maxpower);
//        telemetry.addData("Bezier", Bezier);
//        telemetry.addData("lime", Lime);
//        telemetry.addData("Bezierline", Bezline);
//        telemetry.addData("Tng", BezierCurve.tangential);
//        telemetry.addData("Front left current", frontLeft.motor.motor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Front right current", frontRight.motor.motor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Back left current", backLeft.motor.motor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Back right current", backRight.motor.motor.getCurrent(CurrentUnit.AMPS));

    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector();

        updateMotors();
    }

    public void stopMotors() {
        powerVector = new Vector(0, 0, 0);

        updateMotors();
    }

    public double getDistance(Pose2d p1, Pose2d p2) {
        return Math.sqrt((p1.getX() - p2.getX()) * (p1.getX() - p2.getX()) + (p1.getY() - p2.getY()) * (p1.getY() - p2.getY()));
    }

    public enum RunMode {
        PID, Vector, VectorAndSlows
    }
}