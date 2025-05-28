package org.firstinspires.ftc.teamcode.roboDrive.math;

public class Vector {

    private double x, y, z;
    private double angle,magnitude;

    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        angle =getAngle();
        magnitude =getMagnitude();
    }

    public Vector(double x, double y) {
        this(x, y, 0);
        angle =getAngle();
        magnitude =getMagnitude();
    }





    public Vector() {
        this(0, 0, 0);
    }

    public Vector(Vector other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    public static Vector fromAngleAndMagnitude(double angle, double m) {
        return new Vector(m * Math.cos(angle), m * Math.sin(angle));
    }


    public double getMagnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double getMagnitudeXY() {
        return Math.sqrt(x * x + y * y);
    }



    public double getAngle(){
        return Math.atan2(y, x);
    }

    public Vector plus(Vector other) {
        return new Vector(x + other.x, y + other.y, z + other.z);
    }

    public void scaleToMagnitude(double targetMagnitude) {
        double currentMagnitude = getMagnitude();
        scaleBy(1.0 / currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a) {
        x *= a;
        y *= a;
        z *= a;
    }
    public Vector scaledBy(double a) {
        return new Vector(x * a, y * a, z * a);
    }

    public Vector scaledToMagnitude(double targetMagnitude) {
        Vector aux = new Vector(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public static Vector rotateBy(Vector vector, double theta) {
        return new Vector(
                Math.cos(theta) * vector.getX() + Math.sin(theta) * vector.getY(),
                Math.cos(theta) * vector.getY() - Math.sin(theta) * vector.getX()
        );
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    @Override
    public String toString() {
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }

}
