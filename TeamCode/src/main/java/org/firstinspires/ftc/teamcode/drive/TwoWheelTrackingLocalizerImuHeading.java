package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.newEminescu.modules.drive.SexyLocalizerV2;
import org.firstinspires.ftc.teamcode.roboDrive.Wrappers.CoolIMU;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizerImuHeading extends TwoTrackingWheelLocalizer { // localizator care nu mai tine de drive
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688976378; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 4.134;// X is the up and down direction
    public static double PARALLEL_Y = -0.1968; // Y is the strafe direction

    public static double PERPENDICULAR_X = 7.401;
    public static double PERPENDICULAR_Y = 2.519;
    public static double X_MULTIPLIER = 1.002; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    public CoolIMU imu;

    public TwoWheelTrackingLocalizerImuHeading(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        imu = new CoolIMU(hardwareMap);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "IntakeMotor"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public void startIMUThread(SexyLocalizerV2 loc, LinearOpMode opMode){
        imu.startIMUThread(opMode, loc);
    }

    @Override
    public double getHeading() {
        return imu.getHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return imu.getVelocity();
    }

    /* Lines 77-97 in TwoWheelTrackingLocalizer.java */
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
