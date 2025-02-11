package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class Slider extends SubsystemBase {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    private final Telemetry telemetry;
    private final PIDController positionPIDController;
    private double targetPositon = 0.0;

    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderRightMotorId);
        leftMotor = hardwareMap.get(DcMotorEx.class, Ids.sliderLeftMotorId);

        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        positionPIDController = new PIDController(1.0, 0.0, 0.0);
    }

    public double getRightEncoderPosition() {
        return rightMotor.getCurrentPosition() * Constants.Slider.encoderConversionFactor *
                (Constants.Slider.rightEncoderReversed ? -1.0 : 1.0);
    }

    public double getLeftEncoderPosition() {
        return leftMotor.getCurrentPosition() * Constants.Slider.encoderConversionFactor *
                (Constants.Slider.leftEncoderReversed ? -1.0 : 1.0);
    }
    public void setPower(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }


    public void goToPosition(double position) {
        double rightPosition = getRightEncoderPosition();
        double leftPosition = getLeftEncoderPosition();

        double velocity = positionPIDController.calculate(leftPosition, position);
        //double leftVelocity = positionPIDController.calculate(leftPosition, position);

        rightMotor.setPower(velocity);
        leftMotor.setPower(velocity);

    }

    public boolean isAtPosition(double setPoint) {
        double position = getLeftEncoderPosition();
        return setPoint + 0.5 >= position && position >= setPoint - 0.5;
    }

    public void setTargetPositon(double position) {
        targetPositon = position;
    }

    public void goToHomePosition() {
        setTargetPositon(Constants.Slider.homePosition);
    }

    public void goToSpecimenPosition() {
        setTargetPositon(Constants.Slider.specimenScorePosition);
    }

    public void goToBasketPosition() {
        setTargetPositon(Constants.Slider.basketScorePosition);
    }

    public void goToIntakePosition() {
        setTargetPositon(Constants.Slider.intakePosition);
    }

    @Override
    public void periodic() {
        goToPosition(targetPositon);
    }

    public void stopMotors() {
        setPower(0.0);
    }

    public void motorsData() {
        telemetry.addData("RightMotor", getRightEncoderPosition());
        telemetry.addData("LeftMotor", getLeftEncoderPosition());
    }

    public void motorsVelocity() {
        telemetry.addData("RightMotor", rightMotor.getVelocity());
        telemetry.addData("LeftMotor", leftMotor.getVelocity());
    }

    /*public void motorsData() {
        telemetry.addData("RightMotor", rightMotor.getCurrentPosition());
        telemetry.addData("LeftMotor", leftMotor.getCurrentPosition());
    }*/
}
