package com.ma5951.utils.JoystickContainer;

import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MA_Ps5Controller {
    public final PS4Controller controller;

    public final JoystickButton SquareButton;
    public final JoystickButton TriangleButton;
    public final JoystickButton CrossButton;
    public final JoystickButton CircleButton;

    public final POVButton POVUp;
    public final POVButton POVDown;
    public final POVButton POVLeft;
    public final POVButton POVRight;

    public final JoystickButton OptionsButton;
    public final JoystickButton ShareButton;

    public final JoystickButton stickLeft;
    public final JoystickButton stickRight;

    public final JoystickButton L1;
    public final JoystickButton R1;
    public final JoystickButton L3;
    public final JoystickButton R3;

    public final JoystickButton Left_stick_X;
    public final JoystickButton Right_stick_x;
    public final JoystickButton Left_stick_Y;
    public final JoystickButton Right_stick_y;
    public final JoystickButton PS_L2 ;
    public final JoystickButton PS_R2;

    public MA_Ps5Controller(int ID) {
        controller = new PS4Controller(ID);

        SquareButton = new JoystickButton(controller, RobotConstants.square);
        CircleButton = new JoystickButton(controller, RobotConstants.circle);
        TriangleButton = new JoystickButton(controller, RobotConstants.triangle);
        CrossButton = new JoystickButton(controller, RobotConstants.cross);

        POVUp = new POVButton(controller, RobotConstants.POV_UP);
        POVDown = new POVButton(controller, RobotConstants.POV_DOWN);
        POVLeft = new POVButton(controller, RobotConstants.POV_LEFT);
        POVRight = new POVButton(controller, RobotConstants.POV_RIGHT);

        ShareButton = new JoystickButton(controller, RobotConstants.share);
        OptionsButton = new JoystickButton(controller, RobotConstants.options);

        stickLeft = new JoystickButton(controller, RobotConstants.STICK_LEFT);
        stickRight = new JoystickButton(controller, RobotConstants.STICK_RIGHT);

        L1 = new JoystickButton(controller, RobotConstants.L1);
        R1 = new JoystickButton(controller, RobotConstants.R1);
        L3 = new JoystickButton(controller, RobotConstants.L3);
        R3 = new JoystickButton(controller, RobotConstants.R3);

        PS_L2 = new JoystickButton(controller, RobotConstants.L2);
        PS_R2 = new JoystickButton(controller, RobotConstants.R2);
        Left_stick_X = new JoystickButton(controller, RobotConstants.Left_stick_X);
        Right_stick_x = new JoystickButton(controller, RobotConstants.Right_stick_x);
        Left_stick_Y = new JoystickButton(controller, RobotConstants.Left_stick_Y);
        Right_stick_y = new JoystickButton(controller, RobotConstants.Right_stick_y);

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
