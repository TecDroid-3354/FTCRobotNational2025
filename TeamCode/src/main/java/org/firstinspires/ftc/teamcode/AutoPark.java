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

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.FConstants;
import org.firstinspires.ftc.teamcode.Constants.PedroPathingConstants.LConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

@Autonomous(name = "AutoPark", group = "Examples")
public class AutoPark extends CommandOpMode {

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
    private final Pose goToPiece1 = new Pose(-38, -40, Math.toRadians(80.0));

    private final Pose goFrontToPiece1 = new Pose(-38, -10, Math.toRadians(80.0));
    private final Pose goLeftToPiece1 = new Pose(-48, -10, Math.toRadians(80.0));
    private final Pose leavePiece1 = new Pose(-48, -58, Math.toRadians(80.0));

    private final Pose goFrontToPiece2 = new Pose(-48, -10, Math.toRadians(80.0));
    private final Pose goLeftToPiece2 = new Pose(-57, -10, Math.toRadians(80.0));
    private final Pose leavePiece2 = new Pose(-57, -58, Math.toRadians(80.0));

    private final Pose goFrontToPiece3 = new Pose(-57, -10, Math.toRadians(80.0));
    private final Pose goLeftToPiece3 = new Pose(-61, -10, Math.toRadians(80.0));
    private final Pose leavePiece3 = new Pose(-61, -58, Math.toRadians(80.0));

    private final Pose park = new Pose(65, -56, Math.toRadians(80.0));



    private Path goToBasketPath;
    private PathChain leavePiecePC;

    public void buildPaths() {
        // Go to basket
        goToBasketPath = new Path(new BezierLine(new Point(startPose), new Point(goToBasket)));
        goToBasketPath.setLinearHeadingInterpolation(startPose.getHeading(), goToBasket.getHeading());

        // leave pieces
        leavePiecePC = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goToBasket), new Point(goToPiece1)))
                .setLinearHeadingInterpolation(goToBasket.getHeading(), goToPiece1.getHeading())

                // leave piece 1
                .addPath(new BezierLine(new Point(goToPiece1), new Point(goFrontToPiece1)))
                .setLinearHeadingInterpolation(goToPiece1.getHeading(), goFrontToPiece1.getHeading())
                .addPath(new BezierLine(new Point(goFrontToPiece1), new Point(goLeftToPiece1)))
                .setLinearHeadingInterpolation(goFrontToPiece1.getHeading(), goLeftToPiece1.getHeading())
                .addPath(new BezierLine(new Point(goLeftToPiece1), new Point(leavePiece1)))
                .setLinearHeadingInterpolation(goLeftToPiece1.getHeading(), leavePiece1.getHeading())

                // leave piece 2
                .addPath(new BezierLine(new Point(leavePiece1), new Point(goFrontToPiece2)))
                .setLinearHeadingInterpolation(leavePiece1.getHeading(), goFrontToPiece2.getHeading())
                .addPath(new BezierLine(new Point(goFrontToPiece2), new Point(goLeftToPiece2)))
                .setLinearHeadingInterpolation(goFrontToPiece2.getHeading(), goLeftToPiece2.getHeading())
                .addPath(new BezierLine(new Point(goLeftToPiece2), new Point(leavePiece2)))
                .setLinearHeadingInterpolation(goLeftToPiece2.getHeading(), leavePiece2.getHeading())

                // leave piece 2
                .addPath(new BezierLine(new Point(leavePiece2), new Point(goFrontToPiece3)))
                .setLinearHeadingInterpolation(leavePiece2.getHeading(), goFrontToPiece3.getHeading())
                .addPath(new BezierLine(new Point(goFrontToPiece3), new Point(goLeftToPiece3)))
                .setLinearHeadingInterpolation(goFrontToPiece3.getHeading(), goLeftToPiece3.getHeading())
                .addPath(new BezierLine(new Point(goLeftToPiece3), new Point(leavePiece3)))
                .setLinearHeadingInterpolation(goLeftToPiece3.getHeading(), leavePiece3.getHeading())

                // Go front final
                .addPath(new BezierLine(new Point(leavePiece3), new Point(park)))
                .setLinearHeadingInterpolation(leavePiece3.getHeading(), park.getHeading())

                .build();

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
                if(!follower.isBusy()) {
                    follower.followPath(leavePiecePC);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    quesadillaPosition();

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

    public boolean semiQuesadillaPosition() {
        arm.goToPosition(Constants.Arm.homePositon);
        wrist.goToPosition(Constants.Wrist.homePositon);
        sliderAngle.setTargetPosition(2.7);
        slider.goToIntakePosition();
        gripperAngle.goToIntakePosition();
        gripper.close();

        return slider.isAtPosition(Constants.Slider.intakePosition);
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