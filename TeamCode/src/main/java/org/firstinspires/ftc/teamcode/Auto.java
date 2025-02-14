package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.FConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.LConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

@Autonomous(name = "Auto", group = "Examples")
public class Auto extends CommandOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    boolean hasScoreAPiece = false;

    SliderAngle sliderAngle;
    Slider slider;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    // Leave pices points
    private final Pose startPose = new Pose(-40, -60.0, Math.toRadians(90.0));
    private final Pose goToBasket = new Pose(-55, -51, Math.toRadians(50.0));
    private final Pose goToPiece1 = new Pose(-51, -42, Math.toRadians(90.0));

    private Path goToBasketPath;
    private Path goToPiece1Path;

    public void buildPaths() {
        // Go to basket
        goToBasketPath = new Path(new BezierLine(new Point(startPose), new Point(goToBasket)));
        goToBasketPath.setLinearHeadingInterpolation(startPose.getHeading(), goToBasket.getHeading());

        // Go to piece 1
        goToPiece1Path = new Path(new BezierLine(new Point(goToBasket), new Point(goToPiece1)));
        goToPiece1Path.setLinearHeadingInterpolation(goToBasket.getHeading(), goToPiece1.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);

                follower.followPath(goToBasketPath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (basketScore()) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    intakePosition();
                    follower.followPath(goToPiece1Path);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    intakePosition();

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void initialize() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        com.pedropathing.util.Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        sliderAngle = new SliderAngle(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);

        gripper = new Gripper(hardwareMap, telemetry);
        gripperAngle = new GripperAngle(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // Start instances
        opmodeTimer.resetTimer();
        setPathState(0);
        gripper.close();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Slider", slider.getLeftEncoderPosition());
            telemetry.update();

            run();
        }
        reset();
    }

    public boolean semiQuesadillaPosition() {
        arm.goToPosition(Constants.Arm.homePositon);
        wrist.goToPosition(Constants.Wrist.homePositon);
        sliderAngle.setTargetPosition(2.7);
        slider.goToIntakePosition();
        gripperAngle.goToIntakePosition();
        gripper.close();

        return slider.isAtPosition(Constants.Slider.intakePosition);
    }

    public boolean intakePosition() {
        slider.goToHomePosition();

        if (slider.isAtPosition(Constants.Slider.homePosition)) {
            arm.goToPosition(Constants.Arm.intakePosition);
            wrist.goToPosition(Constants.Wrist.intakePosition);
            sliderAngle.goToQuesadillaPosition();
            slider.goToHomePosition();
            gripperAngle.goToIntakePosition();
            gripper.close();
        }

        return sliderAngle.isAtPosition(Constants.SliderAngle.quesadillaPosition);
    }
    public boolean basketScore() {
        if (!slider.isAtPosition(Constants.Slider.basketAutoScorePosition)) {
            arm.goToPosition(Constants.Arm.basketScorePosition);
            wrist.goToPosition(Constants.Wrist.basketScorePosition);
            slider.setTargetPositon(Constants.Slider.basketAutoScorePosition);
            sliderAngle.goToHomePosition();

        } else {
            // activate the servos to leave the piece
            arm.goToPosition(Constants.Arm.basketAutoScorePosition);
            wrist.goToPosition(Constants.Wrist.basketAutoScorePosition);

            // stop the slider motors to avoid crashing because the delays
            slider.stopMotors();
            try {Thread.sleep(800);} catch (InterruptedException e) {}
            gripper.open();
            try {Thread.sleep(400);} catch (InterruptedException e) {}
            arm.goToPosition(Constants.Arm.basketScorePosition);
            wrist.goToPosition(Constants.Wrist.basketScorePosition);
            try {Thread.sleep(500);} catch (InterruptedException e) {}

            return true;

        }

        return false;
    }

    /*public boolean basketScore() {
        if (!hasScoreAPiece) {
            if (!slider.isAtPosition(Constants.Slider.basketAutoScorePosition)) {
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);

                if (sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                    slider.setTargetPositon(Constants.Slider.basketAutoScorePosition);
                } else {
                    sliderAngle.goToHomePosition();
                }
            } else {
                // activate the servos to leave the piece
                arm.goToPosition(Constants.Arm.basketAutoScorePosition);
                wrist.goToPosition(Constants.Wrist.basketAutoScorePosition);

                // stop the slider motors to avoid crashing because the delays
                slider.stopMotors();
                try {Thread.sleep(800);} catch (InterruptedException e) {}
                gripper.open();
                try {Thread.sleep(400);} catch (InterruptedException e) {}
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);
                try {Thread.sleep(500);} catch (InterruptedException e) {}

                hasScoreAPiece = true;

            }
        } else {
            slider.goToHomePosition();

            if (slider.isAtPosition(Constants.Slider.homePosition)) {
                hasScoreAPiece = false;
                return true;
            }
        }

        return false;
    }*/

    public void grabPiece() {
        // Grab the piece
        gripper.open();
        arm.goToPosition(3.0);
        wrist.goToPosition(2.0);
        slider.goToIntakePosition();
        sliderAngle.goToPosition(6.0);
        try {Thread.sleep(2000);} catch (InterruptedException e) {}
        gripper.close();
        try {Thread.sleep(500);} catch (InterruptedException e) {}

        // Go to quesadilla position
        arm.goToPosition(Constants.Arm.homePositon);
        wrist.goToPosition(Constants.Wrist.homePositon);
    }
}