package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the CurvedBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector. Remember to test your tunings on StraightBackAndForth as well, since tunings
 * that work well for curves might have issues going in straight lines.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous (name = "Curved Back And Forth", group = "PIDF Testing")
public class CurvedBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

    private boolean forward = true;
    public ElapsedTime timp;
    public static double decelarare=3;
    private Follower follower;
    private Pose start = new Pose(40,69);
    private Path forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        backwards = new Path(new BezierCurve( new Point(11.972, 22.947, Point.CARTESIAN),
                new Point(38.245, 25.774, Point.CARTESIAN),
                new Point(40.000, 69.000, Point.CARTESIAN))
        );
        backwards.setConstantHeadingInterpolation(0);
        forwards = new Path(new BezierCurve(new Point(40.000, 69.000, Point.CARTESIAN) ,new Point(38.245, 25.774, Point.CARTESIAN), new Point(11.972, 22.947, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        forwards.setZeroPowerAccelerationMultiplier(decelarare);
        backwards.setZeroPowerAccelerationMultiplier(decelarare);
        follower.setStartingPose(start);
        follower.followPath(forwards);
        timp =new ElapsedTime();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a curve going " + DISTANCE + " inches"
                            + " to the left and the same number of inches forward. The robot will go"
                            + "forward and backward continuously along the path. Make sure you have"
                            + "enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            if(timp.milliseconds()>1000) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }
        }
        else
            timp.reset();

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
