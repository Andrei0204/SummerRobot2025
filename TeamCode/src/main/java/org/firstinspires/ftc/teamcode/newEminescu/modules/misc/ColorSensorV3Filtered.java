package org.firstinspires.ftc.teamcode.newEminescu.modules.misc;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;

public class ColorSensorV3Filtered {

    public NormalizedColorSensor sensor;
    public double a = 0.9;

    public double r, g, b, h, s, v;

    boolean isWhite = false,
            isGreen = false,
            isYellow = false,
            isPurple = false;

    double greenH = 132,
            whiteH = 182,
            purpleH = 216.92,
            yellowH = 97.14;

    double greenS = 0.75,
            whiteS = 0.517,
            purpleS = 0.65,
            yellowS = 0.677;

    double greenV = 0.078,
            whiteV = 0.22,
            purpleV = 0.157,
            yellowV = 0.122;


    double hTolerance = 10,
            sTolerance = 0.05,
            vTolerance = 0.05;


    public boolean isPixel = false;

    final float[] hsvValues = new float[3];

    private final LowPassFilter redFilter, greenFilter, blueFilter;

    NormalizedRGBA colors;

    public ColorSensorV3Filtered(NormalizedColorSensor sensor) {
        this.sensor = sensor;

        colors = sensor.getNormalizedColors();

        redFilter = new LowPassFilter(a, colors.red);
        greenFilter = new LowPassFilter(a, colors.green);
        blueFilter = new LowPassFilter(a, colors.blue);
    }

    public void updateRGB() {

        colors = sensor.getNormalizedColors();

        r = redFilter.getValue(colors.red);
        g = greenFilter.getValue(colors.green);
        b = blueFilter.getValue(colors.blue);

        Color.colorToHSV(colors.toColor(), hsvValues);

        h = hsvValues[0];
        s = hsvValues[1];
        v = hsvValues[2];

        isWhite = isWhite();
        isGreen = isGreen();
        isPurple = isPurple();
        isYellow = isYellow();

        isPixel = isGreen || isPurple || isWhite || isYellow  || (h>=10);

    }

    public boolean isWhite() {
        return (Math.abs(h - whiteH) <= hTolerance
                && Math.abs(v - whiteV) <= vTolerance
                && (Math.abs(s - whiteS) <= sTolerance)
        );
    }

    public boolean isGreen() {
        return (Math.abs(h - greenH) <= hTolerance
                && Math.abs(v - greenV) <= vTolerance
                && (Math.abs(s - greenS) <= sTolerance)
        );
    }

    public boolean isYellow() {
        return (Math.abs(h - yellowH) <= hTolerance
                && Math.abs(v - yellowV) <= vTolerance
                && (Math.abs(s - yellowS) <= sTolerance)
        );
    }

    public boolean isPurple() {
        return (Math.abs(h - purpleH) <= hTolerance
                && Math.abs(v - purpleV) <= vTolerance
                && (Math.abs(s - purpleS) <= sTolerance)
        );
    }

    public void telemetry(Telemetry t) {
        t.addLine()
                .addData("Red", "%.3f", r)
                .addData("Green", "%.3f", g)
                .addData("Blue", "%.3f", b);
        t.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        t.addData("Alpha", "%.3f", colors.alpha);

        t.addLine()
                .addData("isWhite", isWhite)
                .addData("isPurple", isPurple)
                .addData("isGreen", isGreen)
                .addData("isYellow", isYellow)
                .addData("isOverallThere", isPixel);

    }

}
