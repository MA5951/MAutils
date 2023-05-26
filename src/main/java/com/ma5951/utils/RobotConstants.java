/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ma5951.utils;

public class RobotConstants {
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
    public final static double KSTALL_TORQUE_FALCON = 4.68;

    public final static int KNEO_ENCODER_TPR = 42;
    public final static int KCTRE_MAG_ENCODER_TPR = 4096;
    public final static int KCIMCODER_TPR = 80;

    public final static double KDELTA_TIME = 0.02;
    public final static double KGRAVITY_ACCELERATION = 9.8;

    public final static int KDRIVING_JOYSTICK_PORT = 0;
    public final static int KOPERATING_JOYSTICK_PORT = 1;

    public final static int KTRIGGER_THRESHOLD_TIME = 0;

    public static class xbox {
        public static class Buttons {
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
        }
        public static class Axis {
            public static final int STICK_RIGHT_Y_AXIS = 5; 
            public static final int STICK_RIGHT_X_AXIS = 4;
            public static final int STICK_LEFT_Y_AXIS = 1;
            public static final int STICK_LEFT_X_AXIS = 2;
            public static final int L_TRIGER = 2;
            public static final int R_TRIGER = 3;
        }
    }

    public static class PS5 {
        public static class Buttons{
            public static final int CROSS = 2;
            public static final int CIRCLE = 3;
            public static final int SQUARE = 1;
            public static final int TRIANGLE = 4;
            public static final int L1 = 5;
            public static final int R1 = 6;
            public static final int L2 = 7;
            public static final int R2 = 8;
            public static final int SHARE = 9;
            public static final int OPTIONS = 10;
            public static final int L3 = 11;
            public static final int R3 = 12;
            public static final int PS = 13;
            public static final int TOUCHPAD = 14;
        }
        public static class Axis {
            public static final int LEFT_STICK_X = 0;
            public static final int RIGHT_STICK_X = 2;
            public static final int LEFT_STICK_Y = 1;
            public static final int RIGHT_STICK_Y = 5;
            public static final int L2 = 3;
            public static final int R2 = 4;
        }
    }

    public static class POVButtons {
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

    public static final double MAX_VOLTAGE = 12;

}