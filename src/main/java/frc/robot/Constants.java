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
        public static final double MAX_JERK = 0.99;
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
        public static final int maxVel = 8;
        public static final int maxAccel = 5;
    }
    
    public static final class ShooterConstants {
        // TODO fill
        // n | meters | rpm | deg | notes
        // 1 |   ?    |  ?  |  ?  | closest shot
        // 2 |   ?    |  ?  |  ?  | tarmac shot
        // 3 |   ?    |  ?  |  ?  | 
        // 4 |   ?    |  ?  |  ?  | 
        // 5 |   ?    |  ?  |  ?  | hanger shot

        // 145 in | max power (1) | 0 deg

        // quartic regressions
        public static final double getHoodAngle(double distance) {
            return 12;
        }

        public static final double getRPM(double distance) {
            return 40;
        }
        // default shot (tarmac line) TODO set
        public static final double TARMAC_RPM = 0;
        public static final double TARMAC_ANGLE = 0;
        // fender shot (low hub) TODO set
        public static final double LOW_RPM = 0;
        public static final double LOW_ANGLE = 0;
        // ball ejection detection
        public static final int DELTA_VELOCITY_THRESHOLD = 20;
        // timeouts
        public static final int SINGLE_BALL_TIMEOUT = 3;
        public static final int DOUBLE_BALL_TIMEOUT = 6;
        // ratios
        public static final double HOOD_GEAR_RATIO = 36.0 / 460.0;
        public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
        // hood bounds
        public static final double MAX_ANGLE = 22;
        // ISSUE hood is difficult to control because of varying external friction
        // relative to its current position
        // Hood FF
        public static final double hS = 0.019979;
        public static final double hV = 0.096612;
        public static final double hA = 0.02141;
        // Hood PID
        // old hP acquired from sysid
        // public static final double hP = 0.14424;
        // new hP is from 2020 constants i guess...
        // this seems like it could make the hood move violently
        public static final double hP = 0.14424;
        public static final double hI = 0;
        public static final double hD = 0.14486;
        // Flywheel FF
        public static final double fS = 0.13419;
        public static final double fV = 0.10694;
        public static final double fA = 0.016833;
        // Flywheel PID
        public static final double fP = 0.012401;
        public static final double fI = 0;
        public static final double fD = 0.01;
    }

    public static final class TurretConstants {
        // ratios
        public static final double GEAR_RATIO = 26.0 / 300.0;
        public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
        // Physical constraints
        public static final double LIMIT = 85;
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
        public static final double maxA = 20; // acceleration
        public static final double maxV = 40; // velocity
    }

    public static final class HopperConstants {
        public static final double SUCK_SPEED = 0.35;
        public static final double ELEVATOR_SPEED = 0.5;
        public static final double MAX_SPEED = 0.8;
    }

    public static final class IntakeConstants { 
        public static final double INTAKE_SPEED = 0.5;
        public static final double WAIT_TIME = 1000; // in milliseconds, used for the ball counter 
    }

    public static final class VisionConstants {
        public static final double HEIGHT_DIFF = 2.08534; // (meters)
        public static final double MOUNT_ANGLE = 30; // (deg)
        // see: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/linear-filter.html#singlepoleiir
        // possibly change to movingAverage
        public static final double TIMESCALE = 0.1;
        public static final double PERIOD = 0.02;
    }
}
