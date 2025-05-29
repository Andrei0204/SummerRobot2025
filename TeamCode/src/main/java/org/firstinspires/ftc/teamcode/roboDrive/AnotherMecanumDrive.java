package org.firstinspires.ftc.teamcode.roboDrive;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.roboDrive.math.FastTrig;
import org.firstinspires.ftc.teamcode.roboDrive.math.Point;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

import org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.MathFunctions;


@Config
public class AnotherMecanumDrive implements IRobotModule {

    public static boolean ENABLED = true;

    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = false, backLeftMotorReversed = false, backRightMotorReversed = false;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.04, 0, 0.002),
            headingPID = new PIDCoefficients(0.013,
                    0.01
                    , 0.001),
            TangHPID = new PIDCoefficients(0.03, 0.04, 0.0015),
            TBezPID = new PIDCoefficients(0.2, 0, 0.01),
            limelightPID = new PIDCoefficients(0.07, 0, 0.0021);
    public boolean Lime = false;
    public static double lateralMultiplier = 1; //2.13
    public final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public final PIDController tpid = new PIDController(0, 0, 0), hpid = new PIDController(0, 0, 0);
    public double velocityThreshold = 1;
    public Vector powerVector = new Vector();
    public Vector targetVector = new Vector();
    public double diff;
    private SexyLocalizer localizer;
    private RunMode runMode;
    private Pose2d targetPose = new Pose2d();
    private BezierPath BezierCurve = new BezierPath();
    private boolean Bezier = false;
    public static double centripetalScaling = 0.0000001;
    public static double mass = 1;
    public static double maxpower = 0;
    Pose2d currentPose;
    boolean Bezline = false;

    public AnotherMecanumDrive(SampleMecanumDrive drive, SexyLocalizer localizer, RunMode runMode, boolean brake) {
        this.runMode = runMode;
        if (!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            return;
        }

        this.localizer = localizer;

        frontLeft = new CoolMotor(drive.leftFront, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(drive.rightFront, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(drive.leftRear, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(drive.rightRear, CoolMotor.RunMode.RUN, backRightMotorReversed);

        if (brake) {
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.runMode = runMode;
    }

    public void setTargetVector(Vector Vector) {
        this.targetVector = Vector;
    }

    public void setTargetVectorAndSlows(Vector vector, double slowTurn, boolean slowBack, boolean slowForward) {

        double yComp = 0, xComp = 0;

        if (slowBack) yComp = -0.2;
        if (slowForward) yComp = 0.2;

        if (Math.abs(slowTurn) > 0.05) {
            xComp = 0.4 * (slowTurn / Math.abs(slowTurn));
        }

        this.targetVector = new Vector(xComp != 0 ? xComp : vector.getX() * 1.1, yComp != 0 ? yComp : vector.getY(), vector.getZ());
    }

    public void setTargetVectorAndSlowss(Vector vector) {

        double yComp = 0, xComp = 0, zComp = 0;
        yComp = vector.getY() < 0 ? Math.max(vector.getY(), -0.32) : Math.min(vector.getY(), 0.32);
        xComp = vector.getX() < 0 ? Math.max(vector.getX(), -0.32) : Math.min(vector.getX(), 0.32);
        zComp = vector.getZ() < 0 ? Math.max(vector.getZ(), -0.32) : Math.min(vector.getZ(), 0.32);
        this.targetVector = new Vector(xComp, yComp, zComp);
    }


    public RunMode getRunMode() {
        return runMode;
    }

    //200h spend on this drive

    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
    }

    public SexyLocalizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(SexyLocalizer localizer) {
        this.localizer = localizer;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d pose) {
        Bezier = false;
        this.targetPose = new Pose2d(pose.getX(), pose.getY(), Math.toRadians(pose.getHeading()));
    }

    public boolean isAtend() {

        return BezierCurve.AtEnd(currentPose) && stopped();
    }

    public void setLineBezier1(Pose2d p0, Pose2d p3) {

        Bezier = true;
        this.BezierCurve = new BezierPath(p0, p0, p3, p3);
        BezierCurve.tangential = false;
        Bezline = true;

    }

    public void setBezier(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        Bezier = true;
        this.BezierCurve = new BezierPath(p0, p1, p2, p3);
        BezierCurve.tangential = false;
        Bezline = false;
    }

    public void setBeziertang(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        Bezier = true;
        this.BezierCurve = new BezierPath(p0, p1, p2, p3);
        BezierCurve.tangential = true;
    }

    public Vector CentripetalF() {
        double curvature = BezierCurve.closestPointCurvature;

        org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector localizerVelocity = new org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector(localizer.getVelocity().getMagnitude(), localizer.getVelocity().getAngle());

        org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector centripetalVector = new org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector(MathFunctions.clamp(centripetalScaling * mass * Math.pow(MathFunctions.dotProduct(localizerVelocity, MathFunctions.normalizeVector(BezierCurve.getClosestPointTangentVector())), 2) * curvature, -1, 1), BezierCurve.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(BezierCurve.getClosestPointNormalVector().getTheta()));

        Vector finalVector = new Vector(centripetalVector.getXComponent(), centripetalVector.getYComponent());

        return finalVector;

    }

    public Vector FinalDriveVector(Vector Centripetal, Vector Drive) {
        if (Centripetal.getMagnitude() > 1)
            Centripetal.scaleToMagnitude(1);
        if (Drive.getMagnitude() > 1)
            Drive.scaleToMagnitude(1);
        Vector pathingpower = new Vector();
        if (Centripetal.getMagnitude() == 1)
            pathingpower = Centripetal;
        else {
            pathingpower = Centripetal.plus(Drive);
        }
        return pathingpower;

    }


    public Vector FinalDriveVector(Vector Centripetal, Vector Drive, Vector TotalVector) {
        if (Centripetal.getMagnitude() > 1)
            Centripetal.scaleToMagnitude(1);
        if (Drive.getMagnitude() > 1)
            Drive.scaleToMagnitude(1);
        TotalVector.scaleToMagnitude(0);
        Vector pathingpower = new Vector();


        if (Centripetal.getMagnitude() == 1)
            pathingpower = Centripetal;
        else {
            if (Centripetal.plus(TotalVector).getMagnitude() > 1) {
                double magnitutescaler = 1 - Centripetal.getMagnitude();
                Drive.scaleToMagnitude(magnitutescaler);
                pathingpower = Centripetal.plus(Drive);
            } else {
                pathingpower = Centripetal.plus(Drive);
                if (pathingpower.plus(TotalVector).getMagnitude() > 1) {
                    double magnitutescaler = 1 - Centripetal.getMagnitude();
                    Drive.scaleToMagnitude(magnitutescaler);
                    pathingpower = Centripetal.plus(Drive);
                } else {
                    pathingpower = Centripetal.plus(Drive);

                }
            }


        }
        return pathingpower;
    }

    public boolean reachedTarget(double tolerance) {
        if (runMode == RunMode.Vector) return false;
        return getDistance(localizer.getPoseEstimate(), targetPose) <= tolerance;// && stopped();
    }

    public boolean reachedTargetAndStopped(double tolerance) {
        return reachedTarget(tolerance) && stopped();
    }

    public boolean reachedHeading(double tolerance) {
        if (runMode == RunMode.Vector) return false;
        diff = (targetPose.getHeading() - localizer.getHeading()) % (2 * PI);
        while (diff > Math.PI) diff -= Math.PI * 2.0;
        while (diff < -Math.PI) diff += Math.PI * 2.0;
        return Math.abs(diff) <= tolerance;
    }

    public double returnT() {
        return BezierCurve.closestPointTValue;
    }

    public boolean stopped() {
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }

    private void updatePowerVector() {
        switch (runMode) {
            case VectorAndSlows:
                powerVector = targetVector;
                break;
            case Vector:
                powerVector = new Vector(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PID:

                Pose2d prediction = localizer.getPredictedPoseEstimate();
                currentPose = new Pose2d(prediction.getX(), prediction.getY(), prediction.getHeading());
                if (Bezier && reachedTarget(5)) {
                    Point Closestpoint = BezierPath.getClosestPoint(currentPose, 100);
                    if (BezierCurve.tangential) {
                        targetPose = new Pose2d(Closestpoint.x, Closestpoint.y, BezierCurve.GetAngleTangent(BezierPath.closestPointTValue));
                    } else {
                        targetPose = new Pose2d(Closestpoint.x, Closestpoint.y, BezierCurve.heading);

                    }
                }
                //POINT2P / PID2POINT
                double xDiff, yDiff;

                if (Bezier) {
                    xDiff = BezierCurve.getClosestPoint(currentPose, 100).x - currentPose.getX();
                    yDiff = BezierCurve.getClosestPoint(currentPose, 100).y - currentPose.getY();
                } else {
                    xDiff = targetPose.getX() - currentPose.getX();
                    yDiff = targetPose.getY() - currentPose.getY();
                }

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                double theta = Math.atan2(yDiff, xDiff) + Math.PI / 4 - currentPose.getHeading();

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                if (Lime) {
                    tpid.setPID(limelightPID.p, limelightPID.i, limelightPID.d);
                    if (reachedTargetAndStopped(2.5))
                        Lime = false;
                } else
                    tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);
                if (Bezier) {
                    tpid.setPID(TBezPID.p, TBezPID.i, TBezPID.d);
                }
               // tpid.setF(0.015);

                double translationalPower = tpid.calculate(-distance, 0);

                if (translationalPower > 1) translationalPower = Math.signum(translationalPower);

                double max = Math.max(Math.abs(FastTrig.cos(theta)), Math.abs(FastTrig.sin(theta)));

                powerVector = new Vector(
                        translationalPower * FastTrig.cos(theta) / max,
                        translationalPower * FastTrig.sin(theta) / max
                );
                if (Bezier && !isAtend() && !Bezline) {

                    Vector Centripetal = CentripetalF();
                    powerVector = FinalDriveVector(Centripetal, powerVector);
                }

                double headingDiff;

                if (Bezier)
                    headingDiff = (Math.toRadians(BezierPath.curve.p3.getHeading()) - currentPose.getHeading()) % (2 * PI);
                else headingDiff = (targetPose.getHeading() - currentPose.getHeading()) % (2 * PI);

                if (headingDiff > PI) headingDiff -= 2.0 * PI;
                if (headingDiff < -PI) headingDiff += 2.0 * PI;

                headingDiff = Math.toDegrees(headingDiff);

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);
                if (BezierCurve.tangential)
                    hpid.setPID(TangHPID.p, TangHPID.i, TangHPID.d);

                double headingPower = hpid.calculate(-headingDiff, 0);

                if (Math.abs(headingPower) > 1) headingPower = Math.signum(headingPower);

                powerVector = new Vector(powerVector.getX(), powerVector.getY(), headingPower);

                if (Math.abs(powerVector.getMagnitude()) + Math.abs(powerVector.getZ()) > 1) {
                    if (Bezier)
                        powerVector.scaleToMagnitude(1);
                    else {
                        powerVector.scaleToMagnitude(Math.sqrt(2));
                    }


                }
                if (maxpower != 0)
                    if (Math.abs(powerVector.getMagnitude()) > maxpower)
                        powerVector.scaleToMagnitude(maxpower);

                break;

        }

    }

    private void updateMotors() {
        switch (runMode) {
            case VectorAndSlows:

                double x = powerVector.getX(),
                        y = powerVector.getY(),
                        rot = powerVector.getZ();

                frontLeft.setPower(y + x + rot);
                frontRight.setPower(y - x - rot);
                backLeft.setPower(y - x + rot);
                backRight.setPower(y + x - rot);
                break;
            case PID:
                double cos = powerVector.getX(),
                        sin = powerVector.getY(),
                        heading = powerVector.getZ();
                frontLeft.setPower(cos - heading);
                frontRight.setPower(sin + heading);
                backLeft.setPower(sin - heading);
                backRight.setPower(cos + heading);
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
        PID, Vector, VectorAndSlows, Bezier
    }
}