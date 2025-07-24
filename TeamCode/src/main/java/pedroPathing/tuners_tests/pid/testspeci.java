package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
@Disabled

@Config
@Autonomous (name = "testautospeci" +
        "", group = "PIDF Testing")
public class testspeci extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

    private boolean forward = true;

    private Follower follower;

    private PathChain forwards;
    private PathChain backwards;
    private final Pose SP1FIN = new Pose(38 , 67, 0);


    private final Pose COLLECT = new Pose(14 , 35, 0);
    private final Pose COLLECTC1 = new Pose(41 , 47, 0);
    private final Pose COLLECTC2 = new Pose(44 , 33, 0);
    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override

    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        backwards=follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SP1FIN),new Point(COLLECTC1),new Point(COLLECTC2),new Point(COLLECT)))
                .setConstantHeadingInterpolation(0)
                .build();
        forwards=follower.pathBuilder()
                .addPath(new BezierLine(new Point(COLLECT),new Point(SP1FIN)))
                .setConstantHeadingInterpolation(0)
                .build();


        follower.followPath(forwards);
        follower.setStartingPose(COLLECT);

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
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
