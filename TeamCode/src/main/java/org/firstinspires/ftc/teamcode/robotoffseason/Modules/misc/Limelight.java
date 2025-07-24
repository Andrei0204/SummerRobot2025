package org.firstinspires.ftc.teamcode.robotoffseason.Modules.misc;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

import java.util.List;
import java.util.Objects;

@Config
public class Limelight {

    public Limelight3A limelight;
    Vector rotatedPose = null;
    public LLResult result = null;

    double x = 0;

    double y = 0;
    public static double distx = 0;
    public static double disty = 0;
    public double lenght = 0;
    public double width = 0;


    public static double i = -1;
    public static double c = 1;
    public static double angle = 32.2;
    public static double heigh = 22.5;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public void pipeline() {
        limelight.pipelineSwitch(5);
    }

    public void pipelineZero() {
        limelight.pipelineSwitch(0);
    }


    //TODO DETECTIE STIU CE ARE!!!;
    public Pose2d getdistanceVector(double heading, String f) {
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                detectie(f);
                rotatedPose = new Vector(distx,
                        disty,
                        0);
                if (x != 0 && y != 0) {
                    rotatedPose = rotatedPose.rotateBy(rotatedPose, heading);
                    return new Pose2d(rotatedPose.getX(), rotatedPose.getY(), 0);
                }
            }

        }
        return new Pose2d(0, 0, 0);
    }

    public Pose2d getdistanceVector(double heading, String f, double i) {
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                detectie(f, i);
                rotatedPose = new Vector(distx,
                        disty,
                        0);
                if (distx != 0 && disty != 0) {
                    rotatedPose = rotatedPose.rotateBy(rotatedPose, heading);
                    return new Pose2d(rotatedPose.getX(), rotatedPose.getY(), 0);
                }
            }

        }
        return new Pose2d(0, 0, 0);
    }

    private void detectie(String f) {
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            if (Objects.equals(dr.getClassName(), f)) {
                x = dr.getTargetYDegreesNoCrosshair();
                y = dr.getTargetXDegreesNoCrosshair();
                distx = heigh / Math.tan(Math.toRadians(angle - i * x));
                disty = Math.sqrt(distx * distx + heigh * heigh) * Math.tan(Math.toRadians(y * c));
                double xc1 = dr.getTargetCorners().get(0).get(0);
                double xc2 = dr.getTargetCorners().get(3).get(0);
                double xc3 = dr.getTargetCorners().get(2).get(0);
                double yc1 = dr.getTargetCorners().get(0).get(1);
                double yc2 = dr.getTargetCorners().get(3).get(1);
                double yc3 = dr.getTargetCorners().get(2).get(1);
                lenght = Math.sqrt((xc1 - xc2) * (xc1 - xc2) + (yc1 - yc2) * (yc1 - yc2));
                width = Math.sqrt((xc2 - xc3) * (xc2 - xc3) + (yc2 - yc3) * (yc2 - yc3));

                return;
            }
        }
    }

    private void detectie(String f, double v) {
        double x1 = 0, y1 = 0, l1 = 0, w1 = 0;
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            if (Objects.equals(dr.getClassName(), f)) {
                x = dr.getTargetYDegreesNoCrosshair();
                y = dr.getTargetXDegreesNoCrosshair();
                x1 = heigh / Math.tan(Math.toRadians(angle - i * x));
                y1 = Math.sqrt(x1 * x1 + heigh * heigh) * Math.tan(Math.toRadians(y * c));
                double xc1 = dr.getTargetCorners().get(0).get(0);
                double xc2 = dr.getTargetCorners().get(3).get(0);
                double xc3 = dr.getTargetCorners().get(2).get(0);
                double yc1 = dr.getTargetCorners().get(0).get(1);
                double yc2 = dr.getTargetCorners().get(3).get(1);
                double yc3 = dr.getTargetCorners().get(2).get(1);
                l1 = Math.sqrt((xc1 - xc2) * (xc1 - xc2) + (yc1 - yc2) * (yc1 - yc2));
                w1 = Math.sqrt((xc2 - xc3) * (xc2 - xc3) + (yc2 - yc3) * (yc2 - yc3));
                if (l1 / w1 > v && Math.abs(y + 13) < 14 && x < 40) {
                    distx = x1;
                    disty = y1;
                    lenght = l1;
                    width = w1;
                    return;
                }
//                else
//                if(l1/w1<0.8)
//                {
//                       distx=x1;
//                    disty=y1;
//                    lenght=l1;
//                    width=w1;
//                    return;
//                }
                else {
                    distx = 0;
                    disty = 0;
                    lenght = 0;
                    width = 0;
                }
            }
        }
        if (disty == 0) {
            x1 = 0;
            y1 = 0;
            l1 = 0;
            w1 = 0;
            List<LLResultTypes.DetectorResult> dr1 = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : dr1) {
                if (Objects.equals(dr.getClassName(), f)) {
                    x = dr.getTargetYDegreesNoCrosshair();
                    y = dr.getTargetXDegreesNoCrosshair();
                    x1 = heigh / Math.tan(Math.toRadians(angle - i * x));
                    y1 = Math.sqrt(x1 * x1 + heigh * heigh) * Math.tan(Math.toRadians(y * c));
                    double xc1 = dr.getTargetCorners().get(0).get(0);
                    double xc2 = dr.getTargetCorners().get(3).get(0);
                    double xc3 = dr.getTargetCorners().get(2).get(0);
                    double yc1 = dr.getTargetCorners().get(0).get(1);
                    double yc2 = dr.getTargetCorners().get(3).get(1);
                    double yc3 = dr.getTargetCorners().get(2).get(1);
                    l1 = Math.sqrt((xc1 - xc2) * (xc1 - xc2) + (yc1 - yc2) * (yc1 - yc2));
                    w1 = Math.sqrt((xc2 - xc3) * (xc2 - xc3) + (yc2 - yc3) * (yc2 - yc3));
                    if (l1 / w1 < 0.6 && Math.abs(y + 13) < 14 && x < 50) {
                        distx = x1;
                        disty = y1;
                        lenght = l1;
                        width = w1;
                        return;
                    } else {
                        distx = 0;
                        disty = 0;
                        lenght = 0;
                        width = 0;
                    }
                }
            }
        }
        x1 = 0;
        y1 = 0;
        l1 = 0;
        w1 = 0;
        List<LLResultTypes.DetectorResult> dr1 = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : dr1) {
            if (Objects.equals(dr.getClassName(), f)) {
                x = dr.getTargetYDegreesNoCrosshair();
                y = dr.getTargetXDegreesNoCrosshair();
                x1 = heigh / Math.tan(Math.toRadians(angle - i * x));
                y1 = Math.sqrt(x1 * x1 + heigh * heigh) * Math.tan(Math.toRadians(y * c));
                double xc1 = dr.getTargetCorners().get(0).get(0);
                double xc2 = dr.getTargetCorners().get(3).get(0);
                double xc3 = dr.getTargetCorners().get(2).get(0);
                double yc1 = dr.getTargetCorners().get(0).get(1);
                double yc2 = dr.getTargetCorners().get(3).get(1);
                double yc3 = dr.getTargetCorners().get(2).get(1);
                l1 = Math.sqrt((xc1 - xc2) * (xc1 - xc2) + (yc1 - yc2) * (yc1 - yc2));
                w1 = Math.sqrt((xc2 - xc3) * (xc2 - xc3) + (yc2 - yc3) * (yc2 - yc3));
                    distx = x1;
                    disty = y1;
                    lenght = l1;
                    width = w1;
                    return;

            }
        }
    }


    public Pose2d getDEGREES(String f, double robotpos) {
        result = limelight.getLatestResult();
        if (result != null) {
            x = result.getTyNC();
            y = result.getTxNC();
            distx = heigh / Math.tan(Math.toRadians(angle - i * x));
            disty = Math.sqrt(distx * distx + heigh * heigh) * Math.tan(Math.toRadians(y * c));
            return new Pose2d(y, x, 0);
        }
        return new Pose2d(0, 0, 0);
    }

    public double isLateral(String f) {
        if (lenght / width > 0.65 ||
                lenght == width)
            return 0;
        else if (lenght > width)
            return 1;
        else
            return 2;
    }

    public double isLateralylwblue(String f) {
        if (lenght / width > 0.64 ||
                lenght == width)
            return 0;
        else if (lenght > width)
            return 1;
        else
            return 2;
    }
}


