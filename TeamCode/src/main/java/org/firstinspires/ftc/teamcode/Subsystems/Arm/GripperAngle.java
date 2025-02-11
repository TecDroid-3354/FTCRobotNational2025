package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class GripperAngle extends SubsystemBase {
    private Telemetry telemetry;
    private final Servo servo;

    public boolean isLateral = true;

    public GripperAngle(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        servo = hardwareMap.get(Servo.class, Ids.gripperAngleServo);
    }

    public void goToPosition(double position) {
        servo.setPosition(position / 180.0);
    }

    public void goToLateralPosition() {
        goToPosition(Constants.Gripper.lateralPosition);
        isLateral = true;
    }
    public void goToVerticalPosition() {
        goToPosition(Constants.Gripper.verticalPosition);
        isLateral = false;
    }

    public void goToIntakePosition() {
        goToPosition(Constants.Gripper.intakePosition);
        isLateral = true;
    }

    public void goToEstimatedPosition(double angle) {
        double position = ((75.0 / 90.0) * angle) + 55.0;
        //telemetry.addData("Estimated Position", position);
        goToPosition(angle);
    }

    public void goToSpecimenScorePosition() {
        goToPosition(Constants.Gripper.specimenScorePositon);
        isLateral = true;
    }

    public double getPosition() {
        return servo.getController().getServoPosition(3) * 180.0;
    }

    public void servoInfo(){
        telemetry.addData("Gripper Info:", this.getPosition());
    }
}
