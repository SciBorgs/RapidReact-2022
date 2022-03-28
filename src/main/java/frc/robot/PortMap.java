package frc.robot;

public class PortMap {
  //*****************JOYSTICKS*****************//

  public static final int JOYSTICK_LEFT = 1;
  public static final int JOYSTICK_RIGHT = 0;

  //*******************SPARKS******************//
  public static final int HOOD_SPARK = 28;
  public static final int FLYWHEEL_LEFT_SPARK = 0;
  public static final int FLYWHEEL_RIGHT_SPARK = 0;
  public static final int TURRET_SPARK = 3;

  //*******************SWITCHES******************//
  //public static final int TOP_LIMIT_SWITCH = 3;
    public static final int LIMIT_SWITCH_INTAKE = -1;

    // *******************HOPPER******************//

    public static final int HOPPER_SUCK_SPARK = 28;
    public static final int HOPPER_ELEVATOR_SPARK = -1;

    // *******************PNEMUATICS******************//
    public static final int COMPRESSOR = -1;

  //******************ENCODERS******************//
   public static final int TURRET_ENCODER = 2;
   public static final int HOOD_ENCODER = 42;
}
