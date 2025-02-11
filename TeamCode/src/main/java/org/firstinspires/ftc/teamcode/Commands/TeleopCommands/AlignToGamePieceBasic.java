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

public class AlignToGamePieceBasic extends CommandBase {

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

    public AlignToGamePieceBasic(MecanumDrivetrain mecanumDrivetrain, ColorDetector colorDetector, Gamepad gamepad,
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
        arm.goToPosition(Constants.Arm.intakePosition);
        wrist.goToPosition(Constants.Wrist.intakePosition);
        sliderAngle.goToIntakePosition();
        slider.goToIntakePosition();

        // Define the closest detection
        ColorBlobLocatorProcessor.Blob closestDetection = colorDetector.getClosestDetection();

        // Know if we have a detection and get the closest
        if (closestDetection != null) {
            try {
                // define the center point and the angle of the detection
                Point closestDetectionPoint = new Point(closestDetection.getBoxFit().center.x, closestDetection.getBoxFit().center.y);
                mecanumDrivetrain.alignToGamePiece(closestDetectionPoint);
            } catch (RuntimeException ignored) {}
        }
    }

    @Override
    public boolean isFinished() {
        return !gamepad.left_bumper;
    }

}
