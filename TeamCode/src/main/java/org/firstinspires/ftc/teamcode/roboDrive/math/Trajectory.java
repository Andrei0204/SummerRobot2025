package org.firstinspires.ftc.teamcode.roboDrive.math;

import java.util.ArrayList;

public class Trajectory {

    private ArrayList<TrajectorySegment> segments = new ArrayList<>();

    public Trajectory(TrajectorySegment segment){
        segments.add(segment);
    }

    public Trajectory(ArrayList<TrajectorySegment> segments){
        this.segments = segments;
        init();
    }

    private int numberOfSegments;
    private double length;

    private void init(){
        numberOfSegments = segments.size();
        for (TrajectorySegment segment: segments) length+=segment.getLength();
    }

    public double getLength(){
        return length;
    }

    public double getLengthAt(double t){
        if(t >= 1) return length;
        if(t <= 0) return 0;
        double ans = 0;
        t = t * (double) numberOfSegments;
        for(int i = 0;i<(int)t;i++) ans += segments.get(i).getLength();
        ans += segments.get((int)t).getLengthAt(t - (double)((int)t));
        return ans;
    }

    public Pose getPose(double t){
        if(t >= 1) return segments.get(numberOfSegments - 1).getPose(1);
        if(t <= 0) return segments.get(0).getPose(0);
        t = t * (double) numberOfSegments;
        return segments.get((int)t).getPose(t - (double)((int)t));
    }

    private double getDistanceFromPoint(Pose pose, double t){
        if(t < 0 || t > 1) return Double.POSITIVE_INFINITY;
        return new Vector(pose.getX(), pose.getY()).plus(new Vector(-getPose(t).getX(), -getPose(t).getY())).getMagnitude();
    }

    public Vector getTangentVelocity(double t){
        if(t < 0 || t >= 1) return new Vector();
        t *= (double) numberOfSegments;
        return segments.get((int)t).getTangentVelocity(t - (double)((int)t));
    }

    public double getCurvature(double t){
        if(t <= 0 || t >= 1) return 0;
        t *= (double) numberOfSegments;
        return segments.get((int)t).getCurvature(t - (double)((int)t));
    }
}