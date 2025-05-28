package org.firstinspires.ftc.teamcode.roboDrive.math;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class Pose2dM extends Point {

    public double heading;

    public Pose2dM(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }
    public Pose2dM(Point p, double heading) {
        this(p.x, p.y, heading);
    }
    public Pose2dM(float v){
        this(0, 0, 0);
    }

    public Pose2dM subtract(Pose2dM other) {
        return new Pose2dM(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public Pose2dM divide(Pose2dM other) {
        return new Pose2dM(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }


    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.2f", x, y, heading);
    }

    public double getHeading() {
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
