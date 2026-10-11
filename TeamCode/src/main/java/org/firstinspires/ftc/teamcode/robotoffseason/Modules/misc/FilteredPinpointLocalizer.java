package org.firstinspires.ftc.teamcode.robotoffseason.Modules.misc;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roboDrive.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

public class FilteredPinpointLocalizer {

    private final boolean ENABLED = true;
    protected Pose2d currentPosition, currentPredictedPosition;

    private final double filterParameter = 0.9;

    public static double X_ODO_OFFSET = -150,
            Y_ODO_OFFSET = 242;

    public static double xDecelerationConstant = 10, yDecelerationConstant = 10;

    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public Vector glideDelta = new Vector(), velocityVector = new Vector();


    goBildaPPModified ppOdometry;

    public FilteredPinpointLocalizer(HardwareMap hwMap){
        ppOdometry = hwMap.get(goBildaPPModified.class,"odo");

        ppOdometry.setOffsets(
                X_ODO_OFFSET,
                Y_ODO_OFFSET
        ); //todo

        ppOdometry.setEncoderResolution(goBildaPPModified.GoBildaOdometryPods.REVOLUX_35MM_POD);

        ppOdometry.setEncoderDirections(
                goBildaPPModified.EncoderDirection.REVERSED,
                goBildaPPModified.EncoderDirection.FORWARD
        ); //todo check if ok

        ppOdometry.resetPosAndIMU(); //robot has to stay 0.25 sec in place for this to work
    }

    public void reset(){
        ppOdometry.resetPosAndIMU();
    }

    public void setPose(Pose2d pose) {
        ppOdometry.setPosition(
                new Pose2D(
                        DistanceUnit.CM,
                        pose.getX(),
                        pose.getY(),
                        RADIANS,
                        Math.toRadians(pose.getHeading())
                )
        );
        this.currentPosition = pose;
    }

    public Pose2d getCurrentPosition(){
        return currentPosition;
    }

    public Pose2d getCurrentPredictedPosition(){
        return currentPredictedPosition;
    }

    public double getHeading(){
        return currentPosition.getHeading();
    }

    public Vector getVelocity(){
        return velocityVector;
    }

    public double getFrequency(){
        return ppOdometry.getFrequency();
    }

    public void update(){

        ppOdometry.update();

        Pose2D pose = ppOdometry.getPosition();

        currentPosition = new Pose2d(
                pose.getX(DistanceUnit.CM),
                pose.getY(DistanceUnit.CM),
                pose.getHeading(DEGREES)
        ); // current position

        velocityVector = new Vector(
                xVelocityFilter.getValue(ppOdometry.getVelX()),
                yVelocityFilter.getValue(ppOdometry.getVelY())
        );

        Vector predictedGlideVector = new Vector(
                Math.signum(velocityVector.getX()) * velocityVector.getX() * velocityVector.getX() / (2.0 * xDecelerationConstant),
                Math.signum(velocityVector.getY()) * velocityVector.getY() * velocityVector.getY() / (2.0 * yDecelerationConstant)
        );

        glideDelta = Vector.rotateBy(predictedGlideVector, -currentPosition.getHeading());

        currentPredictedPosition = new Pose2d(
                currentPosition.getX() + glideDelta.getX(),
                currentPosition.getY() + glideDelta.getY(),
                currentPosition.getHeading()
        );
    }
}
