/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.MAUtils2;

public class RobotConstants {

    public enum ENCODER {
        No_Encoder, Encoder, Alternate_Encoder
    }

    public enum MOTOR_CONTROLL {
        TALON, VICTOR, SPARKMAXBrushless, SPARKMAXBrushled, Falcon
    }

    protected static int ID1 = 1, ID2 = 2, ID3 = 3, ID4 = 4, ID5 = 5, ID6 = 6, ID7 = 7, ID8 = 8, ID9 = 9, ID10 = 10,
            ID11 = 11, ID12 = 12, ID13 = 13, ID14 = 14, ID15 = 15, ID16 = 16;

    public final static int KMAX_RPM_CIM = 5330;
    public final static int KMAX_RPM_RS = 18730;
    public final static int KMAX_RPM_BAG = 13180;
    public final static int KMAX_RPM_MINI_CIM = 5840;
    public final static int KMAX_RPM_NEO = 5676;
    public final static int KMAX_RPM_775PRO = 18730;

    public final static double KSTALL_TORQUE_CIM = 2.41;
    public final static double KSTALL_TORQUE_RS = 0.38;
    public final static double KSTALL_TORQUE_BAG = 0.4;
    public final static double KSTALL_TORQUE_MINI_CIM = 1.4;
    public final static double KSTALL_TORQUE_NEO = 2.6;
    public final static double KSTALL_TORQUE_775PRO = 0.71;

    public final static int KNEO_ENCODER_TPR = 42;
    public final static int KCTRE_MAG_ENCODER_TPR = 4096;
    public final static int KCIMCODER_TPR = 80;

    public final static int KTICKS_PER_PULSE = 1;
    public final static double KDELTA_TIME = 0.020;
    public final static double KGRAVITY_ACCELERATION = 9.82;

    public final static int KLEFT_JOYSTICK_PORT = 0;
    public final static int KRIGHT_JOYSTICK_PORT = 1;
    public final static int KOPERATING_JOYSTICK_PORT = 2;

    public final static int KTRIGGER_THRESHOLD_TIME = 0;

    // Pneumatics
    public final static int P_ID0 = 0;
    public final static int P_ID1 = 1;
    public final static int P_ID2 = 2;
    public final static int P_ID3 = 3;
    public final static int P_ID4 = 4;
    public final static int P_ID5 = 5;
    public final static int P_ID6 = 6;
    public final static int P_ID7 = 7;

    // DIO
    public final static int DIO_ID0 = 0;
    public final static int DIO_ID1 = 1;
    public final static int DIO_ID2 = 2;
    public final static int DIO_ID3 = 3;
    public final static int DIO_ID4 = 4;
    public final static int DIO_ID5 = 5;
    public final static int DIO_ID6 = 6;
    public final static int DIO_ID7 = 7;
    public final static int DIO_ID8 = 8;
    public final static int DIO_ID9 = 9;

    // Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int BACK = 7;
    public static final int START = 8;
    public static final int STICK_LEFT = 9;
    public static final int STICK_RIGHT = 10;

    // axis
    public static final int STICK_RIGHT_Y_AXIS = 5; // TODO
    public static final int STICK_RIGHT_X_AXIS = 4; // TODO
    public static final int STICK_LEFT_Y_AXIS = 1; // TODO
    public static final int STICK_LEFT_X_AXIS = 2; // TODO
    public static final int L_TRIGER = 2; // TODO
    public static final int R_TRIGER = 3; // TODO

    // POV
    public static final int POV_CENTER = -1;
    public static final int POV_UP = 0;
    public static final int POV_UP_RIGHT = 45;
    public static final int POV_RIGHT = 90;
    public static final int POV_DOWN_RIGHT = 135;
    public static final int POV_DOWN = 180;
    public static final int POV_DOWN_LEFT = 225;
    public static final int POV_LEFT = 270;
    public static final int POV_LEFT_UP = 315;

}