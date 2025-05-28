package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.roboDrive.math.FastTrig;
import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;
import org.opencv.core.Mat;

/**
 * This is the Pinpoint class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry set up with the IMU to have more accurate heading
 * readings. The diagram below, which is modified from Road Runner, shows a typical set up.
 *
 * The view is from the top of the robot looking downwards.
 *
 * left on robot is the y positive direction
 *
 * forward on robot is the x positive direction
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||           |
 *    | ||           |  ----> left (y positive)
 *    |              |
 *    |              |
 *    \--------------/
 *           |
 *           |
 *           V
 *    forward (x positive)
 * With the pinpoint your readings will be used in mm
 * to use inches ensure to divide your mm value by 25.4
 * @author Logan Nash
 * @author Havish Sripada 12808 - RevAmped Robotics
 * @author Ethan Doak - Gobilda
 * @version 1.0, 10/2/2024
 */
@Config
public class PinpointLocalizer {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose2d startPose;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose2d currentVelocity;
    private Pose2d previousPinpointPose;
    private double heading = 0;
    private FastTrig meth;

    Pose ceva;

    public static double xOffset = -147.5, yOffset = 242;

    /**
     * This creates a new PinpointLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public PinpointLocalizer(HardwareMap map){ this(map, new Pose2d());}

    /**
     * This creates a new PinpointLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public PinpointLocalizer(HardwareMap map, Pose2d setStartPose){
        hardwareMap = map;

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        meth = new FastTrig();

        //The default units are inches, but you can swap the units if you wish.
        //If you have already tuned the TwoWheelLocalizer, you can simply use the forwardEncoderPose's y value and strafeEncoderPose's x values.
        setOffsets(xOffset, yOffset, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        //TODO: Tune urself if needed
//        odo.setYawScalar(1.0);

//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(37.2513512516);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        resetPinpoint();

        setStartPose(setStartPose);
        heading = setStartPose.getHeading();
        totalHeading = 0;
        timer = new NanoTimer();
        previousPinpointPose = new Pose2d();
        currentVelocity = new Pose2d();
        deltaTimeNano = 1;
        previousHeading = setStartPose.getHeading();
    }

    public PinpointLocalizer(HardwareMap map, Pose2d setStartPose, boolean fieldCentric){
        hardwareMap = map;

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        meth = new FastTrig();

        //The default units are inches, but you can swap the units if you wish.
        //If you have already tuned the TwoWheelLocalizer, you can simply use the forwardEncoderPose's y value and strafeEncoderPose's x values.
        setOffsets(xOffset, yOffset, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        //TODO: Tune urself if needed
//        odo.setYawScalar(1.0);

//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(37.2513512516);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if(!fieldCentric){
            resetPinpoint();
        }

        setStartPose(new Pose2d(setStartPose.getX(),setStartPose.getY(),odo.getHeading()));
        totalHeading = 0;
        timer = new NanoTimer();
        previousPinpointPose = new Pose2d();
        currentVelocity = new Pose2d();
        deltaTimeNano = 1;
        previousHeading = setStartPose.getHeading();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */

    public Pose2d getPose() {
        return addPoses(startPose, rotatePose(previousPinpointPose, startPose.getHeading(), false));
    }

    public Pose2d getCurrentPosition(){
        return subtractPoses(getPose(), new Pose2d(4-4* meth.cos(getHeading()), -5*meth.sin(getHeading()), 0));
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */

    public Pose2d getVelocity() {
        return currentVelocity;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */

    public org.firstinspires.ftc.teamcode.roboDrive.math.Vector getVelocityVector() {
        return new Vector(currentVelocity.getX(), currentVelocity.getY());
    }

    public double getAngularVelocity(){
        return currentVelocity.getHeading();
    }

    public double getHeading(){
        return heading;
    }

    /**
     * This sets the start pose. Since nobody should be using this after the robot has begun moving,
     * and due to issues with the PinpointLocalizer, this is functionally the same as setPose(Pose).
     *
     * @param setStart the new start pose
     */

    public void setStartPose(Pose2d setStart) {
        this.startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */

    public void setPose(Pose2d setPose) {
        Pose2d setNewPose = subtractPoses(setPose, startPose);
        odo.setPosition(new Pose2D(DistanceUnit.CM, setNewPose.getX(), setNewPose.getY(), AngleUnit.RADIANS, setNewPose.getHeading()));
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */

    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();
        odo.update();
        Pose2D pinpointPose = odo.getPosition();
        Pose2d currentPinpointPose = new Pose2d(pinpointPose.getX(DistanceUnit.CM), pinpointPose.getY(DistanceUnit.CM), pinpointPose.getHeading(AngleUnit.RADIANS));
        heading = currentPinpointPose.getHeading();
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading);
        previousHeading = currentPinpointPose.getHeading();
        Pose2d deltaPose = subtractPoses(currentPinpointPose, previousPinpointPose);
        currentVelocity = new Pose2d(deltaPose.getX() / (deltaTimeNano / Math.pow(10.0, 9)), deltaPose.getY() / (deltaTimeNano / Math.pow(10.0, 9)), deltaPose.getHeading() / (deltaTimeNano / Math.pow(10.0, 9)));
        previousPinpointPose = currentPinpointPose;
    }

    public static Pose2d subtractPoses(Pose2d p1, Pose2d p2){
        return new Pose2d(p1.getX()-p2.getX(), p1.getY()-p2.getY(), p1.getHeading()-p2.getHeading());
    }

    public static Pose2d addPoses(Pose2d p1, Pose2d p2){
        return new Pose2d(p1.getX()+p2.getX(), p1.getY()+p2.getY(), p1.getHeading()+p2.getHeading());
    }

    public static Pose2d rotatePose(Pose2d pose, double theta, boolean rotateHeading) {
        double x = pose.getX() * Math.cos(theta) - pose.getY() * Math.sin(theta);
        double y = pose.getX() * Math.sin(theta) + pose.getY() * Math.cos(theta);
        double heading = rotateHeading ? normalizeAngle(pose.getHeading() + theta) : pose.getHeading();

        return new Pose2d(x, y, heading);
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians;
        while (angle < 0) angle += 2 * Math.PI;
        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */

    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
     * @return returns the Y encoder value
     */

    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    /**
     * This returns the X encoder value as none of the odometry tuners are required for this localizer
     * @return returns the X encoder value
     */

    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    /**
     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
     * @return returns the yaw scalar
     */

    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    /**
     * This sets the offsets and converts inches to millimeters
     * @param xOffset How far to the side from the center of the robot is the x-pod? Use positive values if it's to the left and negative if it's to the right.
     * @param yOffset How far forward from the center of the robot is the y-pod? Use positive values if it's forward and negative if it's to the back.
     * @param unit The units that the measurements are given in
     */
    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
    }

    /**
     * This resets the IMU. Does not change heading estimation.
     */

    public void resetIMU() throws InterruptedException {
        odo.recalibrateIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This resets the pinpoint.
     */
    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void resetPosAndIMU(){
        odo.resetPosAndIMU();
    }
}