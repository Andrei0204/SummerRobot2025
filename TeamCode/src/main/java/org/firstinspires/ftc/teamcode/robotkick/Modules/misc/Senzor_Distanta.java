package org.firstinspires.ftc.teamcode.robotkick.Modules.misc;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Senzor_Distanta {

    public DistanceSensor sensorDistanceback;
    public DistanceSensor sensorDistancefront;
    public Senzor_Distanta(HardwareMap hardwareMap) {
        sensorDistanceback = hardwareMap.get(DistanceSensor.class, "DS");
        sensorDistancefront = hardwareMap.get(DistanceSensor.class, "DSfront");

    }

   public double getDistanceback(){
        return sensorDistanceback.getDistance(DistanceUnit.MM);
   }
    public double getDistancefront(){
        return sensorDistancefront.getDistance(DistanceUnit.MM);
    }
}


