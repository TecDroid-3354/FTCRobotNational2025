package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class Wrist extends SubsystemBase {
    private Telemetry telemetry;
    private final Servo servo;

    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        servo = hardwareMap.get(Servo.class, Ids.wristServo);
    }

    public void goToPosition(double position) {
        servo.setPosition(position / 180.0);
    }

    public double getPosition() {
        return servo.getController().getServoPosition(2) * 180.0;
    }

    public void servoInfo(){
        telemetry.addData("Wrist Info", this.getPosition());
    }
}
