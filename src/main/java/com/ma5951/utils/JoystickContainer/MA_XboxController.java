package com.ma5951.utils.JoystickContainer;

import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MA_XboxController {
    public final XboxController controller;

    public final JoystickButton AButton;
    public final JoystickButton BButton;
    public final JoystickButton YButton;
    public final JoystickButton XButton;

    public final POVButton POVUp;
    public final POVButton POVDown;
    public final POVButton POVLeft;
    public final POVButton POVRight;

    public final JoystickButton backButton;
    public final JoystickButton startButton;

    public final JoystickButton stickLeft;
    public final JoystickButton stickRight;

    public final JoystickButton LB;
    public final JoystickButton RB;

    public final TriggerL triggerL;
    public final TriggerR triggerR;

    public MA_XboxController(int ID) {
        controller = new XboxController(ID);

        AButton = new JoystickButton(controller, RobotConstants.A);
        BButton = new JoystickButton(controller, RobotConstants.B);
        YButton = new JoystickButton(controller, RobotConstants.Y);
        XButton = new JoystickButton(controller, RobotConstants.X);

        POVUp = new POVButton(controller, RobotConstants.POV_UP);
        POVDown = new POVButton(controller, RobotConstants.POV_DOWN);
        POVLeft = new POVButton(controller, RobotConstants.POV_LEFT);
        POVRight = new POVButton(controller, RobotConstants.POV_RIGHT);

        backButton = new JoystickButton(controller, RobotConstants.BACK);
        startButton = new JoystickButton(controller, RobotConstants.START);

        stickLeft = new JoystickButton(controller, RobotConstants.STICK_LEFT);
        stickRight = new JoystickButton(controller, RobotConstants.STICK_RIGHT);

        LB = new JoystickButton(controller, RobotConstants.LB);
        RB = new JoystickButton(controller, RobotConstants.RB);

        triggerL = new TriggerL();
        triggerR = new TriggerR();
    }

    class TriggerL extends Trigger {
        @Override
        public boolean get() {
            return controller.getRawAxis(RobotConstants.L_TRIGER) > 0.5;
        }
    
    }
    
    class TriggerR extends Trigger {
        @Override
        public boolean get() {
            return controller.getRawAxis(RobotConstants.R_TRIGER) > 0.5;
        }
    }
}
