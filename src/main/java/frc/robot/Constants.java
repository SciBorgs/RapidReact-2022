// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
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
        
        // PID (for left and motor groups)
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        // Feedforward
        public static final double kS = 1; 
        public static final double kV = 1;
        public static final double kA = 1;
        
        // Path Following
        public static final int maxVel = 7;
        public static final int maxAccel = 4;

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
        public static final double TARMAC_RPM = 5676;
        public static final double TARMAC_ANGLE = 0;
        // fender shot (low hub)
        public static final double FENDER_RPM = 2750;
        public static final double FENDER_ANGLE = 0;

        // ball ejection detection
        public static final int DELTA_VELOCITY_THRESHOLD = 20;
        // timeouts
        public static final int FLYWHEEL_RAMP_TIMEOUT = 2;
        public static final int SINGLE_BALL_TIMEOUT = 2;
        public static final int DOUBLE_BALL_TIMEOUT = 6;
        // ratios
        public static final double HOOD_GEAR_RATIO = 36.0 / 460.0;
        public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
        // hood bounds
        public static final double MAX_ANGLE = 22;
        // ISSUE hood is difficult to control because of varying external friction
        // relative to its current position
        // Flywheel FF
        public static final double fS = 0.13419;
        public static final double fV = 0.0017823;
        public static final double fA = 0.00028074;
        // Flywheel PID
        public static final double fP = 0.011322;
        public static final double fI = 0;
        public static final double fD = 0;
    }

    public static final class HoodConstants {
        // Hood PID
        public static final double hP = 0.14424;
        public static final double hI = 0.001;
        public static final double hD = 0.14486;
    }

    public static final class TurretConstants {
        // ratios
        public static final double GEAR_RATIO = 26.0 / 300.0;
        public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
        // Physical constraints
        public static final double LIMIT = 70;
        // Through Bore Encoder offset
        public static final double OFFSET = 0;
        // Feedforward
        public static final double kS = 0.13273;
        public static final double kV = 0.038686;
        public static final double kA = 0.012488;
        // PID
        public static final double kP = 0.1461;
        public static final double kI = 0;
        public static final double kD = 0.0728;
        // Trapezoidal motion profile constraints
        public static final double maxVelocity = 20;
        public static final double maxAccel = 0.75;
        public static final double maxVoltage = 12;
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
        // see: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/linear-filter.html#singlepoleiir
        // possibly change to movingAverage
        public static final double TIMESCALE = 0.5;
        public static final double PERIOD = 0.02;
    }
}
