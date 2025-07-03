package org.firstinspires.ftc.teamcode.robotoffseason.CurbeBezier;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

@Config
public class Bezier {
    private Pose p1= null,p2= null,p3= null,p4= null;
    public static double mass=14;
    private double aproximation=1000;
    public Bezier(Pose p1,Pose p2,Pose p3,Pose p4)
    {
        this.p1=p1;
        this.p2=p2;
        this.p3=p3;
        this.p4=p4;

    }
    public Pose getPoset(double t)
    {
        return new Pose(
                (1-t)*(1-t)*(1-t)*p1.getX() + 3*(1-t)*(1-t)*t*p2.getX() + 3*(1-t)*t*t*p3.getX() + t*t*t*p4.getX(),
                (1-t)*(1-t)*(1-t)*p1.getY() + 3*(1-t)*(1-t)*t*p2.getY() + 3*(1-t)*t*t*p3.getY() + t*t*t*p4.getY()
        );
    }
    public Pose fstDerivativePose(double t){
        return new Pose(
                3*(1-t)*(1-t)*(p2.getX()-p1.getX())+6*(1-t)*t*(p3.getX()-p2.getX()) +3*t*t*(p4.getX()-p3.getX()),
                3*(1-t)*(1-t)*(p2.getY()-p1.getY())+6*(1-t)*t*(p3.getY()-p2.getY()) +3*t*t*(p4.getY()-p3.getY())
                );
    }
    public double fstDerivative(double t){
        Pose derivative= fstDerivativePose(t);
        return derivative.getX()/derivative.getY();
    }
    public Pose scndDderivativePose(double t)
    {
        return new Pose(
                6*(t-1)*(p3.getX()-2*p2.getX()+p1.getX()) +6*t*(p4.getX()-2*p3.getX() +p2.getX()),
                6*(t-1)*(p3.getY()-2*p2.getY()+p1.getY()) +6*t*(p4.getY()-2*p3.getY() +p2.getY())
        );
    }
    public double scndDerivative(double t){
        Pose derivative= scndDderivativePose(t);
        return derivative.getX()/derivative.getY();
    }
    public double getApproximationLenght(){
        double sum=0;

        for(int i =0;i<aproximation;i++)
        {
            Pose firstpoint = getPoset(i/aproximation);
            Pose scndpoint = getPoset((i+1)/aproximation);
            double distance = Math.sqrt( (firstpoint.getX()-scndpoint.getX())*(firstpoint.getX()-scndpoint.getX())
                    +(firstpoint.getY()-scndpoint.getY())*(firstpoint.getY()-scndpoint.getY()));
            sum += distance;
        }
        return sum;
    }
    public double getRadius(double t){
        double r;

        Pose firstDerivative = fstDerivativePose(t),
            secondDerivative = scndDderivativePose(t);

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
    public double getCentripetalForce(double t)
    {
        return mass* fstDerivative(t)*fstDerivative(t)/getRadius(t);
    }
    public Vector getCorrectionVector(double t)
    {
        double pant =-1/fstDerivative(t);
        double angle =Math.atan(pant);
            return new Vector(angle,getCentripetalForce(t));
    }

}
