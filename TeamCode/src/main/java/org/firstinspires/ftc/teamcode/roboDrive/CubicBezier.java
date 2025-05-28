package org.firstinspires.ftc.teamcode.roboDrive;



import com.acmerobotics.roadrunner.geometry.Pose2d;


public class CubicBezier {

    public Pose2d p0, p1, p2, p3;

    double t;
    int resolution = 100;
    public double length = 0;

    public CubicBezier(Pose2d p0, Pose2d p1, Pose2d p2, Pose2d p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.t = 0;
        length = getLength();
    }

    public double getLength(){
        double length = 0;
        for(int i=0;i<resolution; i++){
            Pose2d p1 = getPose(i/resolution);
            Pose2d p2 = getPose((i+1)/resolution);
            length+= Meth.distance(p1, p2);
        }
        return length;
    }

    public double getLengthAt(double t){
        double length = 0;
        for(int i=0; i<t*resolution; i++){
            Pose2d p1 = getPose(i/resolution);
            Pose2d p2 = getPose((i+1)/resolution);
            length+= Meth.distance(p1, p2);
        }
        return length;
    }

    public Pose2d getEndPoint(){
        return this.p3;
    }

    public Pose2d getPose(double t) {
        return new Pose2d(
                Math.pow(1 - t, 3) * p0.getX() + 3 * Math.pow(1 - t, 2) * t * p1.getX() + 3 * (1 - t) * Math.pow(t, 2) * p2.getX() + Math.pow(t, 3) * p3.getX(),
                Math.pow(1 - t, 3) * p0.getY() + 3 * Math.pow(1 - t, 2) * t * p1.getY() + 3 * (1 - t) * Math.pow(t, 2) * p2.getY() + Math.pow(t, 3) * p3.getY(),
                Math.pow(1 - t, 3) * p0.getHeading() + 3 * Math.pow(1 - t, 2) * t * p1.getHeading() + 3 * (1 - t) * Math.pow(t, 2) * p2.getHeading() + Math.pow(t, 3) * p3.getHeading()
        );
    }

    public Pose2d get1stDerivativePose(double t) {
        return new Pose2d(
                3 * Math.pow(1 - t, 2) * (p1.getX() - p0.getX()) + 6 * (1 - t) * t * (p2.getX() - p1.getX()) + 3 * Math.pow(t, 2) * (p3.getX() - p2.getX()),
                3 * Math.pow(1 - t, 2) * (p1.getY() - p0.getY()) + 6 * (1 - t) * t * (p2.getY() - p1.getY()) + 3 * Math.pow(t, 2) * (p3.getY() - p2.getY()),
                3 * Math.pow(1 - t, 2) * (p1.getHeading() - p0.getHeading()) + 6 * (1 - t) * t * (p2.getHeading() - p1.getHeading()) + 3 * Math.pow(t, 2) * (p3.getHeading() - p2.getHeading())
        );
    }

    public double getTangentAngleAt(double t){
        return Math.toDegrees(Math.atan(get1stDerivativePose(t).getY()));
    }

    public Pose2d get2ndDerivativePose(double t) {
        return new Pose2d(
                6 * (1 - t) * (p2.getX() - 2 * p1.getX() + p0.getX()) + 6 * t * (p3.getX() - 2 * p2.getX() + p1.getX()),
                6 * (1 - t) * (p2.getY() - 2 * p1.getY() + p0.getY()) + 6 * t * (p3.getY() - 2 * p2.getY() + p1.getY()),
                6 * (1 - t) * (p2.getHeading() - 2 * p1.getHeading() + p0.getHeading()) + 6 * t * (p3.getHeading() - 2 * p2.getHeading() + p1.getHeading())
        );
    }

    public double getRadius(double t) {
        double r;

        Pose2d firstDerivative = get1stDerivativePose(t),
                secondDerivative = get2ndDerivativePose(t);

        double x1 = firstDerivative.getX(),
                y1 = firstDerivative.getY(),
                x2 = secondDerivative.getX(),
                y2 = secondDerivative.getY();

        double upperTerm = Math.abs(x1 * y2 - y1 * x2);
        double lowerTerm = Math.pow(Math.sqrt(x1 * x1 + y1 * y1), 3);

        double k = upperTerm / lowerTerm;

        r = 1.0 / k;

        return r;
    }

    public double getCentripetalForce(double speed, double t) {
        return speed * speed / getRadius(t);
    }

}