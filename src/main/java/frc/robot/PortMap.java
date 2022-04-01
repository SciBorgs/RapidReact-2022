package frc.robot;

public class PortMap {
    // **************INPUT DEVICES***************//

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX_CONTROLLER = 2;

    // *******************SPARKS******************//
    public static final int LEFT_FRONT_SPARK = 21;
    public static final int LEFT_MIDDLE_SPARK = 22;
    public static final int LEFT_BACK_SPARK = 7;
    public static final int RIGHT_FRONT_SPARK = 14;
    public static final int RIGHT_MIDDLE_SPARK = 10;
    public static final int RIGHT_BACK_SPARK = 26;

    public static final int HOOD_SPARK = 4;
    public static final int FLYWHEEL_LEFT_SPARK = 30;
    public static final int FLYWHEEL_RIGHT_SPARK = 15;
    public static final int TURRET_SPARK = 11;

    // *******************SWITCHES******************//
    public static final int TOP_LIMIT_SWITCH = 3;
    public static final int INTAKE_SWITCH = 6;
    public static final int LIMIT_SWITCH_INTAKE = -1;

    // *******************HOPPER******************//

    public static final int HOPPER_SUCK_SPARK = 16;
    public static final int HOPPER_ELEVATOR_SPARK = 24;

    // *******************MISC******************//
    public static final int PIGEON_ID = 42;
    public static final int LEFT_ENCODER = -1;
    public static final int RIGHT_ENCODER = -1;

    // ******************INTAKE********************//
    // spark 23 = intake flip
    public static final int INTAKE_ARM_FORWARD_CHANNEL = -1;
    public static final int INTAKE_ARM_REVERSE_CHANNEL = -1;
    public static final int INTAKE_SUCK_SPARK = 80;

    // *******************PNEMUATICS******************//
    public static final int COMPRESSOR = -1;

    // *******************CLIMBER******************//
    // nicholas: 19
    // daniel: 20
    public static final int CLIMBER_ARM = -1;
    public static final int HOOK_MOTOR = 80;
    public static final int RETRACT_CLIMBER_ARM = -1;

    // ******************ENCODERS******************//
    public static final int TURRET_ENCODER = 2;
    public static final int HOOD_ENCODER = 3;

    public static final class Joystick {
        public static final int JOYSTICK_TRIGGER = 1;
        public static final int JOYSTICK_CENTER_BUTTON = 2;
        public static final int JOYSTICK_LEFT_BUTTON = 3;
        public static final int JOYSTICK_RIGHT_BUTTON = 4;

        public static final int[][] JOYSTICK_BUTTON_MATRIX_LEFT = { { 5, 6, 7 }, { 10, 9, 8 } };
        public static final int[][] JOYSTICK_BUTTON_MATRIX_RIGHT = { { 13, 12, 11 }, { 14, 15, 16 } };
    }

    public static final class XboxController {
        public static final int XBOX_A = 1;
        public static final int XBOX_B = 2;
        public static final int XBOX_X = 3;
        public static final int XBOX_Y = 4;

        public static final int XBOX_BUMPER_LEFT = 5;
        public static final int XBOX_BUMPER_RIGHT = 6;

        public static final int XBOX_BACK = 7;
        public static final int XBOX_START = 8;

        public static final int XBOX_STICK_LEFT_BUTTON = 9;
        public static final int XBOX_STICK_RIGHT_BUTTON = 10;

        public static final int XBOX_TRIGGER_LEFT = 2;
        public static final int XBOX_TRIGGER_RIGHT = 3;

        public static final int XBOX_LEFT_JOY_X = 0;
        public static final int XBOX_LEFT_JOY_Y = 1;

        public static final int XBOX_RIGHT_JOY_X = 4;
        public static final int XBOX_RIGHT_JOY_Y = 5;
    }
}
