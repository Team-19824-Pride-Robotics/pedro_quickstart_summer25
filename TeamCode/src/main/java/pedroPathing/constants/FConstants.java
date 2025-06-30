package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 7;

        FollowerConstants.xMovement = (54.3 + 52.5 + 53.7) / 3;
        FollowerConstants.yMovement = (26.9 + 26.5 + 26.56) / 3;

        FollowerConstants.forwardZeroPowerAcceleration = (-23 + -21 + -21) / 3;
        FollowerConstants.lateralZeroPowerAcceleration = (-33.95 + -33.7 + -33.6) / 3;

        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.useSecondaryDrivePID = true;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                .1,
                0,
                .01,
                0);
        FollowerConstants.translationalIntegral.setCoefficients(
                0,
                0,
                0,
                0);
        FollowerConstants.translationalPIDFFeedForward = 0.02;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                0.075,
                0,
                0.05,
                0);
        FollowerConstants.secondaryTranslationalIntegral.setCoefficients(
                0,
                0,
                0,
                0);
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.0005;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(
                5,
                0,
                0.5,
                0);
        FollowerConstants.headingPIDFFeedForward = 0.01;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(
                1.5,
                0,
                0.1,
                0);
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.0005;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(
                0.01,
                0,
                0.0001,
                0.6,
                0);
        FollowerConstants.drivePIDFFeedForward = 0.01;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(
                0.02,
                0,
                0.0005,
                0.6,
                0);
        FollowerConstants.secondaryDrivePIDFFeedForward = 0.01;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;


    }
}
