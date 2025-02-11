package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.JoystickCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrivetrain;

@TeleOp(name = "MecanumOnly", group = "Op Mode")
public class MecanumOnly extends CommandOpMode {
    private MecanumDrivetrain mecanumDrivetrain;
    private GamepadEx controller;
    private SparkFunOTOS otos;
    SliderAngle sliderAngle;
    Slider slider;
    Gripper gripper;

    double pos = 0.0;

    @Override
    public void initialize() {

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setAngularUnit(AngleUnit.DEGREES);

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        mecanumDrivetrain.setDefaultCommand(new JoystickCMD(
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.right_stick_x,
                () -> gamepad1.right_stick_button,
                this::getGyroYaw,
                mecanumDrivetrain
        ));

        // Gamepad buttons
        controller = new GamepadEx(gamepad1);
        sliderAngle = new SliderAngle(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
        gripper = new Gripper(hardwareMap, telemetry);

        otos.resetTracking();

        configureButtonBindings();
    }
    public void configureButtonBindings() {
        // reset IMU
        new GamepadButton(controller, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(() -> {
                    otos.resetTracking();
                }));

        new GamepadButton(controller, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    otos.resetTracking();
                }));
    }

    public double getGyroYaw() {
        //return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return otos.getPosition().h;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Y", controller.getLeftY());
            telemetry.update();

            run();
        }
        reset();


    }
}
