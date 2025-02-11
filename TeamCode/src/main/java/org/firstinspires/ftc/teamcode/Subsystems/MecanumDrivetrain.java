package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;
import org.opencv.core.Point;


public class MecanumDrivetrain extends SubsystemBase {
    // Telemetry
    Telemetry telemetry;

    // Motors
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    // Kinematics
    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    // PController
    PIDFController xAlignPIDFController;
    PIDFController yAlignPIDFController;

    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Ids.frontLeftId);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, Ids.backLeftId);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, Ids.frontRightId);
        backRightMotor = hardwareMap.get(DcMotorEx.class, Ids.backRightId);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // PIDF
        frontRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(1.0, 0, 0, Constants.MecanumConstants.kFrontRightF));
        backRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(1.0, 0, 0, Constants.MecanumConstants.kBackRightF));
        frontLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(1.0, 0, 0, Constants.MecanumConstants.kFrontLeftF));
        backLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(1.0, 0, 0, Constants.MecanumConstants.kBackLeftF));

        // Init runing using encoder mode
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // tune the pids apart
        xAlignPIDFController = new PIDFController(0.05, 0.0, 0.0, 0.0);
        yAlignPIDFController = new PIDFController(0.04, 0.0, 0.0, 0.0);
    }

    public void drive(ChassisSpeeds speeds) {
        // Convert to wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(speeds);

        // Get the individual wheel speeds
        double convertionFactor = Constants.MecanumConstants.kMps2Radps;

        double frontLeft = wheelSpeeds.frontLeftMetersPerSecond * convertionFactor;
        double frontRight = wheelSpeeds.frontRightMetersPerSecond * convertionFactor;
        double backLeft = wheelSpeeds.rearLeftMetersPerSecond * convertionFactor;
        double backRight = wheelSpeeds.rearRightMetersPerSecond * convertionFactor;

        frontLeftMotor.setVelocity(frontLeft, AngleUnit.RADIANS);
        frontRightMotor.setVelocity(frontRight, AngleUnit.RADIANS);
        backLeftMotor.setVelocity(backLeft, AngleUnit.RADIANS);
        backRightMotor.setVelocity(backRight, AngleUnit.RADIANS);
    }

    public void basicDrive(double x, double y, double rx) {
        ChassisSpeeds speeds = new ChassisSpeeds(y, x, rx);
        drive(speeds);
    }

    public void driveFieldOriented(double x, double y, double rx, double robotHeading) {

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                y, x, rx, Rotation2d.fromDegrees(robotHeading)
        );
        drive(speeds);
    }

    public boolean alignToGamePiece(Point closestDetection) {
        // Camera Center point
        Point piecePositionCenter = new Point(160, 120);

        // speeds
        ChassisSpeeds speeds;

        // Sample positions
        double xSamplePoint = closestDetection.x;
        double ySamplePoint = closestDetection.y;

        double yVelocity = 0.0;
        double xVelocity = 0.0;

        yVelocity = yAlignPIDFController.calculate(ySamplePoint, piecePositionCenter.y);
        xVelocity = xAlignPIDFController.calculate(xSamplePoint, piecePositionCenter.x);

        speeds = new ChassisSpeeds(yVelocity, xVelocity, 0.0);
        drive(speeds);

        return piecePositionCenter.y + Constants.MecanumConstants.alignTolerance >= ySamplePoint && ySamplePoint >= piecePositionCenter.y - Constants.MecanumConstants.alignTolerance
                && piecePositionCenter.x + Constants.MecanumConstants.alignTolerance >= xSamplePoint && xSamplePoint >= piecePositionCenter.x - Constants.MecanumConstants.alignTolerance;
    }

    public void motorsData() {
        // Velocity
        double factor = Constants.MecanumConstants.kTicksps2mps;
        telemetry.addData("FL mps", frontLeftMotor.getVelocity() * factor);
        telemetry.addData("FR mps", frontRightMotor.getVelocity() * factor);
        telemetry.addData("BL mps", backLeftMotor.getVelocity() * factor);
        telemetry.addData("BR mps", backRightMotor.getVelocity() * factor);
    }
}