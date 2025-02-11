package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.FConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.LConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

@Autonomous(name = "Auto1Piece", group = "Examples")
public class Auto1Piece extends CommandOpMode {

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
    private final Pose startPose = new Pose(-36, -60.0, Math.toRadians(90.0));
    private final Pose goToBasket = new Pose(-55, -55, Math.toRadians(240.0));
    private final Pose goToPiece1 = new Pose(-55, -30.0, Math.toRadians(0.0));



    private Path goToBasketPath;

    public void buildPaths() {
        // Go to basket
        goToBasketPath = new Path(new BezierLine(new Point(startPose), new Point(goToBasket)));
        goToBasketPath.setLinearHeadingInterpolation(startPose.getHeading(), goToBasket.getHeading());

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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
                    if (quesadillaPosition()) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

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
            telemetry.update();

            run();
        }
        reset();
    }

    public boolean quesadillaPosition() {
        arm.goToPosition(Constants.Arm.homePositon);
        wrist.goToPosition(Constants.Wrist.homePositon);
        sliderAngle.goToQuesadillaPosition();
        slider.goToHomePosition();
        gripperAngle.goToIntakePosition();
        gripper.close();

        return sliderAngle.isAtPosition(Constants.SliderAngle.quesadillaPosition);
    }
    public boolean basketScore() {
        if (!hasScoreAPiece) {
            if (!slider.isAtPosition(Constants.Slider.basketScorePosition)) {
                arm.goToPosition(Constants.Arm.basketScorePosition);
                wrist.goToPosition(Constants.Wrist.basketScorePosition);

                if (sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                    slider.goToBasketPosition();
                } else {
                    sliderAngle.goToHomePosition();
                }
            } else {
                sliderAngle.goToBasketPosition();
                if (sliderAngle.isAtPosition(Constants.SliderAngle.basketScorePosition)) {
                    try {Thread.sleep(1200);} catch (InterruptedException e) {}
                    gripper.open();
                    hasScoreAPiece = true;
                }

            }
        } else {
            if (!sliderAngle.isAtPosition(Constants.SliderAngle.homePosition)) {
                sliderAngle.goToHomePosition();
            } else {
                slider.goToHomePosition();

                if (slider.isAtPosition(Constants.Slider.homePosition)) {
                    hasScoreAPiece = false;
                    return true;
                }
            }
        }

        return false;
    }
}