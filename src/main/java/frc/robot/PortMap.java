package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public final class PortMap {

  public static final class InputDevices {
    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX_CONTROLLER = 2;
  }

  public static final class Drivetrain {
    public static final int LEFT_FRONT_SPARK = 22;
    public static final int LEFT_MIDDLE_SPARK = 21;
    public static final int LEFT_BACK_SPARK = 7;
    public static final int RIGHT_FRONT_SPARK = 14;
    public static final int RIGHT_MIDDLE_SPARK = 26;
    public static final int RIGHT_BACK_SPARK = 10;

    public static final int PIGEON = 42;
  }

  public static final class Shooter {
    public static final int HOOD_SPARK = 4;
    public static final int[] FLYWHEEL_SPARKS = {30, 5};

    public static final int[] HOOD_ENCODER_QUADRATURE = {0, 1};
  }

  public static final class Turret {
    public static final int TURRET_SPARK = 11;

    public static final int[] TURRET_ENCODER_QUADRATURE = {3, 4};
  }

  public static final class Hopper {
    public static final int SUCK_SPARK = 16;
    public static final int ELEVATOR_SPARK = 24;
  }

  public static final class Intake {
    // spark 23 = intake flip
    public static final int SUCK_SPARK = 23;

    public static final int[] ARM_CHANNELS = {0, 1};

    public static final int LIMIT_SWITCH = 6;
  }

  public static final class Climber {
    public static final int TELESCOPE_SPARK = 19;
    public static final int ARMS_SPARK = 20;
  }

  public static final class JoystickMap {
    public static final class Button {
      public static final int TRIGGER = 1;
      public static final int CENTER = 2;
      public static final int LEFT = 3;
      public static final int RIGHT = 4;

      public static final int[][] MATRIX_LEFT = {{5, 6, 7}, {10, 9, 8}};
      public static final int[][] MATRIX_RIGHT = {{13, 12, 11}, {14, 15, 16}};
    }

    public static final class Axis {
      public static final int X = Joystick.kDefaultXChannel;
      public static final int Y = Joystick.kDefaultYChannel;
      // i think:
      public static final int TWIST = Joystick.kDefaultTwistChannel;
      public static final int THROTTLE = Joystick.kDefaultThrottleChannel;

      public static final int POV = 0; // TODO: populate
    }
  }

  public static final class XboxControllerMap {
    public static final class Button {
      public static final int A = XboxController.Button.kA.value;
      public static final int B = XboxController.Button.kB.value;
      public static final int X = XboxController.Button.kX.value;
      public static final int Y = XboxController.Button.kY.value;

      public static final int BUMPER_LEFT = XboxController.Button.kLeftBumper.value;
      public static final int BUMPER_RIGHT = XboxController.Button.kRightBumper.value;

      public static final int BACK = XboxController.Button.kBack.value;
      public static final int START = XboxController.Button.kStart.value;

      public static final int STICK_LEFT = XboxController.Button.kLeftStick.value;
      public static final int STICK_RIGHT = XboxController.Button.kRightStick.value;
    }

    public static final class Axis {
      public static final int TRIGGER_LEFT = XboxController.Axis.kLeftTrigger.value;
      public static final int TRIGGER_RIGHT = XboxController.Axis.kRightTrigger.value;

      public static final int LEFT_X = XboxController.Axis.kLeftX.value;
      public static final int LEFT_Y = XboxController.Axis.kLeftY.value;

      public static final int RIGHT_X = XboxController.Axis.kRightX.value;
      public static final int RIGHT_Y = XboxController.Axis.kRightY.value;
    }
  }
}
