package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.Constants.Ids;

public class Arm extends SubsystemBase {
    private Telemetry telemetry;
    private final Servo rightServo;
    private final Servo leftServo;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        rightServo = hardwareMap.get(Servo.class, Ids.rightArmServo);
        leftServo = hardwareMap.get(Servo.class, Ids.leftArmServo);

        leftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void goToPosition(double position) {
        rightServo.setPosition(position / 180.0);

        leftServo.setPosition(position / 180.0);
    }

}
