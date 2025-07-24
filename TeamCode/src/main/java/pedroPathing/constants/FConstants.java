package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14;

        FollowerConstants.xMovement = 79.99129657890144;
        FollowerConstants.yMovement = 63.812485788349434;

        FollowerConstants.forwardZeroPowerAcceleration = -26.08515405921818;
        FollowerConstants.lateralZeroPowerAcceleration = -62.52399299835914;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2,0,0.035,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.1 ,0,0.001 ,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00004 ,0.6,0);//0.015//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0115,0,0.00007 ,0.6,0);//0.015
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0013 ,0.6,0);//0.015//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0115,0,0.00007 ,0.6,0);//0.015
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,
                0.00018
                ,0.6,0); // Not being used, @see useSecondaryDrivePID
//0.012


        FollowerConstants.zeroPowerAccelerationMultiplier = 4.6;//5
        FollowerConstants.centripetalScaling = 0.00035;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.985;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        FollowerConstants.useBrakeModeInTeleOp=true;
//

    }
}
