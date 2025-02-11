package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class JoystickCMD extends CommandBase {

    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier rx;
    private DoubleSupplier robotHeading;

    // field oriented toggle
    private BooleanSupplier fieldOrientedBtn;
    private boolean fieldOrientedActived = true;

    private double velocityFactor = 1.0;

    private MecanumDrivetrain mecanumDrivetrain;

    public JoystickCMD(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx, BooleanSupplier fieldOrientedBtn,
                       DoubleSupplier robotHeading, MecanumDrivetrain mecanumDrivetrain) {
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.robotHeading = robotHeading;
        this.fieldOrientedBtn = fieldOrientedBtn;

        this.mecanumDrivetrain = mecanumDrivetrain;


        addRequirements(mecanumDrivetrain);
    }


    @Override
    public void execute() {
        //toggleFieldOriented();

        double xVelocity = x.getAsDouble() * Constants.MecanumConstants.maxMetersPerSecondDrive * velocityFactor;
        double yVelocity = y.getAsDouble() * Constants.MecanumConstants.maxMetersPerSecondDrive * velocityFactor;
        double rxVelocity = rx.getAsDouble() * Constants.MecanumConstants.maxMetersPerSecondRotation;
        double robotYaw = robotHeading.getAsDouble();

        if (fieldOrientedActived) {
            mecanumDrivetrain.driveFieldOriented(xVelocity, yVelocity, rxVelocity, robotYaw);
        }else {
            mecanumDrivetrain.basicDrive(xVelocity, yVelocity, rxVelocity);
        }

    }

    public void setVelocityFactor(double velocityFactor) {
        this.velocityFactor = velocityFactor;
    }
}