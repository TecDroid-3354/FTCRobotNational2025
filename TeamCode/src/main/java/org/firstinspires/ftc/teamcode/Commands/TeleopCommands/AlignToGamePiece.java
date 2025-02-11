package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Sensors.ColorDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Point;

public class AlignToGamePiece extends CommandBase {

    private boolean hasFindAPiece = false;
    private double angleOfGripper = 90.0;
    private final MecanumDrivetrain mecanumDrivetrain;
    private final ColorDetector colorDetector;
    private final Gamepad gamepad;
    private final Slider slider;
    private final SliderAngle sliderAngle;
    private final Gripper gripper;
    private final GripperAngle gripperAngle;
    private final Arm arm;
    private final Wrist wrist;

    public AlignToGamePiece(MecanumDrivetrain mecanumDrivetrain, ColorDetector colorDetector, Gamepad gamepad,
                            Arm arm,
                            Slider slider,
                            SliderAngle sliderAngle,
                            Gripper gripper,
                            GripperAngle gripperAngle,
                            Wrist wrist) {
        this.mecanumDrivetrain = mecanumDrivetrain;
        this.colorDetector = colorDetector;
        this.gamepad = gamepad;

        this.arm = arm;
        this.slider = slider;
        this.sliderAngle = sliderAngle;
        this.gripper = gripper;
        this.gripperAngle = gripperAngle;
        this.wrist = wrist;
    }

    @Override
    public void execute() {
        // Go to the intake positions if we hasn't find a detection yet
        if (!hasFindAPiece) {
            arm.goToPosition(Constants.Arm.intakePosition);
            wrist.goToPosition(Constants.Wrist.intakePosition);
            sliderAngle.goToIntakePosition();
            slider.goToIntakePosition();
        }

        // Do the intake sequence
        else {
            //gripperAngle.goToEstimatedPosition(angleOfGripper);
            //try {Thread.sleep(650);} catch (InterruptedException e) {}
            grabAPieceSequence();
        }

        // Define the closest detection
        ColorBlobLocatorProcessor.Blob closestDetection = colorDetector.getClosestDetection();

        // Know if we have a detection and get the closest
        if (closestDetection != null) {
            try {
                // define the center point and the angle of the detection
                Point closestDetectionPoint = new Point(closestDetection.getBoxFit().center.x, closestDetection.getBoxFit().center.y);
                angleOfGripper = closestDetection.getBoxFit().angle;

                // align the robot to it
                // With this logic, first is going to check if the robot has a piece, if yes, the robot do the intake sequence
                // Also we put the slider as a condition to avoid the intake sequence when we don't want it
                if (!hasFindAPiece && mecanumDrivetrain.alignToGamePiece(closestDetectionPoint) && sliderAngle.getAbsoluteEncoderPosition() > 1.7){
                    hasFindAPiece = true;
                }
            } catch (RuntimeException e) {}
        }
    }

    public void grabAPieceSequence() {
        gripper.open();
        arm.goToPosition(Constants.Arm.sampleTakePosition);
        wrist.goToPosition(Constants.Wrist.intakePosition);
        try {Thread.sleep(650);} catch (InterruptedException e) {}
        gripper.close();
        try {Thread.sleep(500);} catch (InterruptedException e) {}
        arm.goToPosition(Constants.Arm.intakePosition);
        slider.goToIntakePosition();
        hasFindAPiece = false;
    }

    /*public void grabAPieceSequence() {
        slider.setTargetPositon(Constants.Slider.cameraIntakePosition);
        gripper.open();
        if (slider.getLeftEncoderPosition() > Constants.Slider.cameraIntakePosition - 0.1) { // The 0.1 is the tolerance
            arm.goToPosition(Constants.Arm.sampleTakePosition);
            wrist.goToPosition(Constants.Wrist.intakePosition);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
            }
            gripper.close();
            try {
                Thread.sleep(650);
            } catch (InterruptedException e) {
            }
            arm.goToPosition(Constants.Arm.intakePosition);
            slider.goToIntakePosition();
            hasFindAPiece = false;
        }
    }*/

    @Override
    public boolean isFinished() {
        return !gamepad.left_bumper;
    }

}
