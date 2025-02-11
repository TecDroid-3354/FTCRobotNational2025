package org.firstinspires.ftc.teamcode.Constants;

public class Constants {

    public static final class Ids {
        // Drivetrain
        public static String frontLeftId = "frontLeft";
        public static String frontRightId = "frontRight";
        public static String backLeftId = "backLeft";
        public static String backRightId = "backRight";

        // Imu
        public static String imuId = "imu";

        // OTOS
        public static String otosId = "OTOS";

        // Webcam
        public static String webcamName = "webcam0";

        // Arm
        public static String rightArmServo = "rightArmServo";
        public static String leftArmServo = "leftArmServo";
        public static String wristServo = "wristServo";
        public static String gripperAngleServo = "gripperAngleServo";
        public static String gripperServo = "gripperServo";

        // Slider
        public static String sliderRightMotorId = "sliderRightMotor";
        public static String sliderLeftMotorId = "sliderLeftMotor";

        // Slider angle
        public static String sliderAngleRightMotorId = "sliderAngleRightMotor";
        public static String sliderAngleLeftMotorId = "sliderAngleLeftMotor";

    }

    public static final class MecanumConstants {
        public static double ticksPerRevolution = 425;
        public static double wheelRadius = 0.075;
        public static double kMps2Radps = 1 / wheelRadius;
        public static double kTicksps2mps = (2 * Math.PI * wheelRadius) / ticksPerRevolution;

        public static double maxMetersPerSecondDrive = 3.0;
        public static double maxMetersPerSecondRotation = 3.0;

        // PIDF coeficients

        public static double kFrontLeftF = 20.0;
        public static double kFrontRightF = 12.0;
        public static double kBackLeftF = 13.0;
        public static double kBackRightF = 23.0;

        public static double alignTolerance = 10.0;
    }

    public static final class Arm {
        // Positions
        public static double homePositon = 170.0;
        public static double intakePosition = 60;
        public static double sampleTakePosition = 0.0;
        public static double basketScorePosition = 0.0;

    }

    public static final class Wrist {
        // Positions
        public static double homePositon = 180.0;
        public static double intakePosition = 0.0;
        public static double basketScorePosition = 0.0;

    }

    public static final class Gripper {
        // Positions gripper
        public static double openPositon = 110;
        public static double closePosition = 70;

        // Positions gripper angle
        public static double lateralPosition = 130.0; // 15.0
        public static double verticalPosition = 55; // 75

        public static double intakePosition = 130.0;
        public static double specimenScorePositon = 0.0; // 130.0

    }

    public static final class Slider {
        // Positions
        public static double homePosition = 0.1;
        public static double intakePosition = 2.0;
        public static double cameraIntakePosition = 2.8;

        public static double specimenScorePosition = 1.2; // 2.0

        public static double basketScorePosition = 7.2;

        public static double encoderConversionFactor = 1 / 537.7;
        public static boolean rightEncoderReversed = true;
        public static boolean leftEncoderReversed = true;
    }

    public static final class SliderAngle {
        // Velocity

        // Encoder
        public static double ticksPerRev = 1.0 / 8192.0;


        // Positions
        public static double homePosition = 89; // 550.0
        public static double intakePosition = 1.5; // 55.0
        public static double quesadillaPosition = 6.0;
        public static double posIntakePosition = 0.0; // 55.0

        public static double specimenScorePosition = 35;

        public static double basketScorePosition = 77;
    }


}

