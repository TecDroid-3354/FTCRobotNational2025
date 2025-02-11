package org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.Constants;

// This acts as a method of updating FollowerConstants without direct access to it.
public class FConstants { // This is how we change Follower Constants.
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.OTOS;
        FollowerConstants.mass = 14;
        FollowerConstants.xMovement = 54.5;
        FollowerConstants.yMovement = 20.0;
        FollowerConstants.forwardZeroPowerAcceleration = -137.2;
        FollowerConstants.lateralZeroPowerAcceleration = -146.5;
        //FollowerConstants.zeroPowerAccelerationMultiplier = 0.5;
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.02, 0.0, 0.0, 0.0, 0.0);
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.08, 0.0, 0.006, 0.2);
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.4, 0.0, 0.02, 0.02);

        // PIDs
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.useSecondaryDrivePID = true;

        FollowerConstants.leftFrontMotorName = Constants.Ids.frontLeftId;
        FollowerConstants.leftRearMotorName = Constants.Ids.backLeftId;
        FollowerConstants.rightFrontMotorName = Constants.Ids.frontRightId;
        FollowerConstants.rightRearMotorName = Constants.Ids.backRightId;

        FollowerConstants.leftFrontMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorEx.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorEx.Direction.REVERSE;

    }
}

