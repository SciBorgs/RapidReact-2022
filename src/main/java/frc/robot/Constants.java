// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class PIDConstants {
    double p, i, d;

    public PIDConstants(double p) {
      this(p, 0);
    }

    public PIDConstants(double p, double i) {
      this(p, i, 0);
    }

    public PIDConstants(double p, double i, double d) {
      this.p = p;
      this.i = i;
      this.d = d;
    }

    public PIDController get() {
      return new PIDController(p, i, d);
    }

    public ProfiledPIDController getWithMotionProfile(Constraints constraints) {
      return new ProfiledPIDController(p, i, d, constraints);
    }
  }

  public static class ProfiledPIDConstants {
    PIDConstants pidConstants;
    Constraints constraints;

    public ProfiledPIDConstants(PIDConstants pidConstants, Constraints constraints) {
      this.pidConstants = pidConstants;
      this.constraints = constraints;
    }

    public ProfiledPIDController get() {
      return new ProfiledPIDController(pidConstants.p, pidConstants.i, pidConstants.d, constraints);
    }
  }

  public static class FFConstants {
    private double s, v, a;

    public FFConstants(double s, double v) {
      this(s, v, 0);
    }

    public FFConstants(double s, double v, double a) {
      this.s = s;
      this.v = v;
      this.a = a;
    }

    public SimpleMotorFeedforward get() {
      return new SimpleMotorFeedforward(s, v, a);
    }
  }

  public static final double MARGIN_OF_ERROR = 1e-2;

  public static final class DriveConstants {
    // public static final PIDConstants drivePID = new PIDConstants()

    // for stalling detection
    public static final double CURRENT_THRESHOLD = 0;
    public static final double DUTY_CYCLE_THRESHOLD = 0;
    public static final double VELOCITY_THRESHOLD = 0;
    // for slew rate limiter
    public static final double DELTA = 3.5;
    // ratios
    public static final double GEAR_RATIO = 1 / 13.7;
    public static final double ROBOT_WIDTH = 0.7398; // (meters)
    public static final double WHEEL_RADIUS = 0.0766; // (meters)
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;

    public static final PIDConstants drivePID = new PIDConstants(1);
    public static final FFConstants driveFF = new FFConstants(0.1, 0.1, 0.1);

    public static final PIDConstants turnPID = new PIDConstants(0.01);

    // Path Following
    public static final int maxVel = 4;
    public static final int maxAccel = 2;

    // misc
    public static final double driveBackSpeeds = -0.4;
  }

  public static final class ShooterConstants {
    // TODO fill
    // n | meters | rpm | deg | notes
    // 1 |   ?    |  ?  |  ?  | closest shot
    // 2 |   ?    |  ?  |  ?  | tarmac shot
    // 3 |   ?    |  ?  |  ?  |
    // 4 |   ?    |  ?  |  ?  |
    // 5 |   ?    |  ?  |  ?  | hanger shot

    // quartic regressions
    public static final double getHoodAngle(double distance) {
      return 0;
    }

    public static final double getRPM(double distance) {
      return 200;
    }

    // default shot (tarmac line) TODO set
    public static final double TARMAC_RPM = 4800;
    public static final double TARMAC_ANGLE = 0;
    // fender shot (low hub)
    public static final double FENDER_RPM = 2750;
    public static final double FENDER_ANGLE = 0;

    // timeouts
    public static final int FLYWHEEL_RAMP_TIMEOUT = 2;
    public static final int SINGLE_BALL_TIMEOUT = 2;
    public static final int DOUBLE_BALL_TIMEOUT = 6;

    // ratios
    public static final double TURRET_GEAR_RATIO = 26.0 / 300.0;
    public static final double HOOD_GEAR_RATIO = 36.0 / 460.0;
    public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
    // bounds
    public static final int HOOD_LIMIT = 22; // (deg)
    public static final int TURRET_LIMIT = 70; // (deg)
    // ISSUE hood is difficult to control because of varying external friction
    // relative to its current position

    public static final FFConstants flywheelFF = new FFConstants(0.13419, 0.0017823, 0.00028074);
    public static final PIDConstants flywheelPID = new PIDConstants(0.003, 0.001, 0);

    public static final FFConstants turretFF = new FFConstants(0.13273, 0.038686, 0.012488);
    public static final PIDConstants turretPID = new PIDConstants(0.1461, 0, 0.0728);
    public static final Constraints turretConstraints = new Constraints(20, 0.75);
    public static final ProfiledPIDConstants turretProfiledPID =
        new ProfiledPIDConstants(turretPID, turretConstraints);

    public static final PIDConstants hoodPID = new PIDConstants(0.14424, 0.001, 0.14486);
  }

  public static final class HopperConstants {
    public static final double SUCK_SPEED = 0.35;
    public static final double ELEVATOR_SPEED = 0.65;
    public static final double MAX_SPEED = 0.8;
  }

  public static final class IntakeConstants {
    public static final double INTAKE_SPEED = 0.5;
    public static final double DEBOUNCE_TIME = 0.5; // (seconds)
  }

  public static final class ClimberConstants {
    public static final double TELESCOPE_SPEED = 0.6;
    public static final double ARM_SPEED = 0.5;
  }

  public static final class VisionConstants {
    public static final double HEIGHT_DIFF = 2.08534; // (meters)
    public static final double MOUNT_ANGLE = 30; // (deg)
    // see:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/linear-filter.html#singlepoleiir
    // possibly change to movingAverage
    public static final double TIMESCALE = 0.5;
    public static final double PERIOD = 0.02;
  }
}
