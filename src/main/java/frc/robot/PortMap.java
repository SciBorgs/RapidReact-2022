package frc.robot;

public class PortMap {
    // **************INPUT DEVICES***************//

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX_CONTROLLER = 2;

    //*******************2019********************//
    //*******************SPARKS******************//

    public static final int LEFT_FRONT_SPARK = 6; 
    public static final int LEFT_MIDDLE_SPARK = 5; 
    public static final int LEFT_BACK_SPARK = 4;

    public static final int RIGHT_FRONT_SPARK = 3;
    public static final int RIGHT_MIDDLE_SPARK = 2;
    public static final int RIGHT_BACK_SPARK = 1;

    public static final int LIFT_SPARK = 7;

    //*******************TALONS******************//

    public static final int PIGEON_TALON = 9;

    public static final int ARM_TILT_TALON = 10; // CHANGE TO 10 FOR COMP, 12 FOR PRACTICE
    public static final int INTAKE_TALON = 11; // CHANGE TO 11 FOR COMP, 10 FOR PRACTICE

    public static final int LEFT_ZLIFT = 9;
    public static final int RIGHT_ZLIFT = 8;

    //***************LIMIT*SWITCHES**************//

    public static final int CASCADE_AT_BOTTOM_LIMIT_SWITCH = 1;
    public static final int ARM_AT_TOP_LIMIT_SWITCH = 3;

    //***************DOUBLE*SOLENOIDS*************//

    public static final int[] GEAR_SHIFTER_SOLENOID = {0, 1};
    public static final int[] SECURE_HATCH_SOLENOID = {0, 1};
    public static final int[] ARM_SOLENOID = {4, 5};
    public static final int[] POP_HATCH_SOLENOID = {6, 7};

    public static final int GEAR_SHIFTER_SOLENOID_PDP = 0;
    public static final int SECURE_HATCH_SOLENOID_PDP = 1;
    public static final int ARM_SOLENOID_PDP = 0;
    public static final int POP_HATCH_SOLENOID_PDP = 0;

    // public static final int[] ZLIFT_SOLENOID = {2,3};

    //*******************MISC********************//

    public static final int PRESSURE_SENSOR = 0;
    public static final int TARGETING_LIGHT_DIGITAL_OUTPUT = 5;

    //*******************INPUT********************//
    public static final class Joystick {
        public static final int JOYSTICK_TRIGGER = 1;
        public static final int JOYSTICK_CENTER_BUTTON = 2;
        public static final int JOYSTICK_LEFT_BUTTON = 3;
        public static final int JOYSTICK_RIGHT_BUTTON = 4;

        public static final int[][] JOYSTICK_BUTTON_MATRIX_LEFT = { { 5, 6, 7 }, { 10, 9, 8 } };
        public static final int[][] JOYSTICK_BUTTON_MATRIX_RIGHT = { { 13, 12, 11 }, { 14, 15, 16 } };
    }

    public static final class XboxController {
        // BUTTONS

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

        // AXIS

        public static final int XBOX_TRIGGER_LEFT = 2;
        public static final int XBOX_TRIGGER_RIGHT = 3;

        public static final int XBOX_LEFT_JOY_X = 0;
        public static final int XBOX_LEFT_JOY_Y = 1;

        public static final int XBOX_RIGHT_JOY_X = 4;
        public static final int XBOX_RIGHT_JOY_Y = 5;
    }


    // *******************2022****************** //
    // kept so subsystems don't error out

    //*******************SPARKS******************//

    public static final int HOOD_SPARK = 4;
    public static final int FLYWHEEL_LEFT_SPARK = 30;
    public static final int FLYWHEEL_RIGHT_SPARK = 5;
    public static final int TURRET_SPARK = 11;

    // *******************SWITCHES******************//
    public static final int INTAKE_SWITCH = 6;
    public static final int LIMIT_SWITCH_INTAKE = 5;

    // *******************HOPPER******************//

    public static final int HOPPER_SUCK_SPARK = 16;
    public static final int HOPPER_ELEVATOR_SPARK = 24;

    // *******************MISC******************//
    public static final int PIGEON_ID = 42;

    // ******************INTAKE********************//
    // spark 23 = intake flip
    public static final int INTAKE_ARM_FORWARD_CHANNEL = 0;
    public static final int INTAKE_ARM_REVERSE_CHANNEL = 1;
    public static final int INTAKE_SUCK_SPARK = 23;

    // *******************CLIMsBER******************//
    public static final int CLIMBER_TELESCOPE = 19;
    public static final int CLIMBER_ARMS = 20;

    // ******************ENCODERS******************//
    public static final int TURRET_ENCODER_A = 3;
    public static final int TURRET_ENCODER_B = 4;
    public static final int HOOD_ENCODER_A = 0;
    public static final int HOOD_ENCODER_B = 1;
}
