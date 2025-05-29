package org.firstinspires.ftc.teamcode.roboDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roboDrive.math.Point;
import org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector;


public class BezierPath {

    public static CubicBezier curve;
    private double startHeading;
    private double endHeading;
    public double heading;
    public static double closestPointTValue;
    public static double closestPointCurvature;
    public static org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector closestPointTangentVector;
    public static org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector closestPointNormalVector;
    public boolean tangential;

    public BezierPath(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3, double heading) {
        curve = new CubicBezier(p0, p1, p2, p3);
        this.heading = heading;
    }

    public BezierPath(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        curve = new CubicBezier(p0, p1, p2, p3);
        heading = 0;
    }

    public BezierPath() {

    }

    public static double getLenght() {
        return curve.length;
    }

    public static Pose2d getPoint(double t) {
        return curve.getPose(t);
    }

    public static boolean AtEnd(Pose2d pose) {
        if (((getClosestPoint(pose, 100).x - 1) < getPoint(1).getX() && getPoint(1).getX() < (getClosestPoint(pose, 100).x + 1)) && ((getClosestPoint(pose, 100).y - 1) < getPoint(1).getY() && getPoint(1).getY() < (getClosestPoint(pose, 100).y + 1)))
            return true;
        return false;
    }

    public double GetAngleTangent(double t) {
        return curve.getTangentAngleAt(t);
    }

    public static Point getClosestPoint(Pose2d pose, int searchStepLimit) {
        double lower = 0;
        double upper = 1;
        Pose2d returnPoint;

        // we don't need to calculate the midpoint, so we start off at the 1/4 and 3/4 point
        for (int i = 0; i < searchStepLimit; i++) {
            if (Meth.distance(pose, getPoint(lower + 0.25 * (upper - lower))) > Meth.distance(pose, getPoint(lower + 0.75 * (upper - lower)))) {
                lower += (upper - lower) / 2.0;
            } else {
                upper -= (upper - lower) / 2.0;
            }
        }


        closestPointTValue = lower + 0.5 * (upper - lower);
        closestPointTValue += 0.1;
        closestPointTValue = Math.min(closestPointTValue, 1);
        closestPointTangentVector = new org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector(curve.get1stDerivativePose(closestPointTValue).getX(), curve.get1stDerivativePose(closestPointTValue).getY());
        closestPointNormalVector = new org.firstinspires.ftc.teamcode.pedroPathingold.pathGeneration.Vector(curve.get2ndDerivativePose(closestPointTValue).getX(), curve.get2ndDerivativePose(closestPointTValue).getY());

        closestPointCurvature = curve.getRadius(closestPointTValue);
        returnPoint = getPoint(closestPointTValue);
        return new Point(returnPoint.getX(), returnPoint.getY());
    }

    public Vector getClosestPointTangentVector() {
        return closestPointTangentVector;
    }

    public Vector getClosestPointNormalVector() {
        return closestPointNormalVector;
    }


}