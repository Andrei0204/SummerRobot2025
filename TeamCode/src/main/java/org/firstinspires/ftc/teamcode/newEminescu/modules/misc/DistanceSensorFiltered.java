package org.firstinspires.ftc.teamcode.newEminescu.modules.misc;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;

public class DistanceSensorFiltered {

    public double distance;
    DistanceSensor sensor;
    LowPassFilter filter;
    double a = 0.9;

    public DistanceSensorFiltered(DistanceSensor sensor){
        this.sensor = sensor;

        filter = new LowPassFilter(a, sensor.getDistance(DistanceUnit.CM));
    }

    public double getDistance(){
        return distance;
    }

    public void update(){
        distance = filter.getValue(sensor.getDistance(DistanceUnit.CM));
    }

}