package org.firstinspires.ftc.teamcode.Commands.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.GripperAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.SliderAngle;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

public class BasketScoreCommand extends CommandBase {
    private boolean finish = false;
    Slider slider;
    SliderAngle sliderAngle;
    Gripper gripper;
    GripperAngle gripperAngle;
    Arm arm;
    Wrist wrist;

    public BasketScoreCommand(Arm arm,
                              Slider slider,
                              SliderAngle sliderAngle,
                              Gripper gripper,
                              GripperAngle gripperAngle,
                              Wrist wrist) {

        this.arm = arm;
        this.slider = slider;
        this.gripper = gripper;
        this.gripperAngle = gripperAngle;
        this.sliderAngle = sliderAngle;
        this.wrist = wrist;

    }

    @Override
    public void execute() {
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
                finish = true;
            }

        }

    }

    @Override
    public boolean isFinished() {
        //return sliderAngle.isAtPosition(Constants.SliderAngle.basketScorePosition);
        return finish;
    }

}
