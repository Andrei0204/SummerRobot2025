package org.firstinspires.ftc.teamcode.roboDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;


public class Meth {

    public static double distance(Pose2d one, Pose2d two) {
        return Math.sqrt(Math.pow(one.getX()-two.getX(), 2) + Math.pow(one.getY()-two.getY(), 2));
    }
    public static double dotProduct(Vector one, Vector two) {
        return one.getX()*two.getX() + one.getZ()*two.getY();
    }
    public static double clamp(double num, double lower, double upper) {
        if (num < lower) return lower;
        if (num > upper) return upper;
        return num;
    }
    public static double getSign(double get) {
        if (get == 0) return 0;
        if (get > 0) return 1;
        return -1;
    }

}
