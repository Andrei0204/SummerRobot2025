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
    public static double i = 1;

    public static double camerangle = -33;
    public static double h = 24;
    public static double hsample =4 ;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public void pipeline() {
        limelight.pipelineSwitch(7);
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

    private void detectie(String f) {

            x = result.getTx();
            y = -result.getTy();
            distx =-((h-hsample)/Math.tan(Math.toRadians(y + camerangle)));
            disty = distx*Math.tan(Math.toRadians( (x )));

    }

    public Pose2d getDEGREES(String f,double robotpos) {
        result = limelight.getLatestResult();
        if (result != null) {
                        x = result.getTx();
                        y = -result.getTy();
                        distx =-((h-hsample)/Math.tan(Math.toRadians(y + camerangle)));
                        disty = distx*Math.tan(Math.toRadians( (x )));
                        return new Pose2d(y, x, 0);
                }
        return new Pose2d(0, 0, 0);
    }

    public double isLateral(String f) {
        result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    if (Objects.equals(dr.getClassName(), f)) {
                        double xc1=dr.getTargetCorners().get(0).get(0);
                        double xc2=dr.getTargetCorners().get(3).get(0);
                        double xc3=dr.getTargetCorners().get(2).get(0);
                        double yc1=dr.getTargetCorners().get(0).get(1);
                        double yc2=dr.getTargetCorners().get(3).get(1);
                        double yc3=dr.getTargetCorners().get(2).get(1);
                        double lenght= Math.sqrt((xc1-xc2)*(xc1-xc2) +(yc1-yc2)*(yc1-yc2));
                        double width= Math.sqrt((xc2-xc3)*(xc2-xc3) +(yc2-yc3)*(yc2-yc3));
                        if(lenght>width)
                            return 1;
                        else
                            return 2;
                    }

                }

            }
        }
        return 0;
    }
}


