// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Point;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //*********************** OTHER ***********************//
    public static final double SECONDS_PER_TICK = 0.02;
    public static final double CURRENT_THRESHOLD = 0;
    public static final double DUTY_CYCLE_THRESHOLD = 0;
    public static final double VELOCITY_THRESHOLD = 0;
    
    //***************** GEAR RATIOS ***********************//
    public static final double WHEEL_ENCODER_GEAR_RATIO = 13.7;
    public static final double LEFT_ENCODER_GEAR_RATIO = 13.7;
    public static final double RIGHT_ENCODER_GEAR_RATIO = 13.7;
    // 1.0 / 230.0;

    //***************** FIELD DATA ************************//
    public static final double FIELD_WIDTH = 8.2296;
    public static final double FIELD_LENGTH = 16.4592;
    public static final double BALL_PLACEMENT_RADIUS = 3.885;
    public static final Point POINT_HUB = new Point(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Point[] RED_BALLS = new Point[] {
        new Point(11.633, 2.028), new Point(11.551, 6.3768), new Point(8.999, 7.973),
        new Point(6.084, 7.331), new Point(4.52, 3.263), new Point(9.229, 0.332)};
    public static final Point[] BLUE_BALLS = new Point[] {
        new Point(10.629, 0.958), new Point(12.144, 5.042), new Point(7.368, 7.490),
        new Point(4.968, 6.194), new Point(5.03, 1.864), new Point(7.582, 0.283)};
    
    public static final double FENDER_RADIUS = 1.0;

    //public static final List<Point> PATH_TEST = new PathSmoother(PATH_TEST_RAW, 20).getFinalPath();
    
    public static final class DriveConstants {
        public static final double MAX_JERK = 0.15;
        // ratios
        public static final double GEAR_RATIO = 1 / 13.7;
        public static final double ROBOT_WIDTH = 0.7398; // (meters) 
        public static final double DISTANCE_PER_PULSE = 1; // TODO update
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
        // quartic regressions
        public static final double getHoodAngle(double distance) {
            return 0;
        }

        public static final double getRPM(double distance) {
            return 0;
        }
        
        public static final int DELTA_VOLTAGE_THRESHOLD = 20;
        // ratios
        public static final double HOOD_GEAR_RATIO = 36.0 / 460.0;
        public static final double DISTANCE_PER_PULSE = 1.0 / 2048.0;
        // Through Bore Encoder offset
        public static final double OFFSET = 32.5;
        public static final double MAX = 22;
        // Hood FF
        public static final double hS = 0;
        public static final double hV = 0;
        public static final double hA = 0;
        // Hood PID
        public static final double hP = 6.0/360;
        public static final double hI = 0;
        public static final double hD = 0;
        // Flywheel FF
        public static final double fS = 0;
        public static final double fV = 0;
        public static final double fA = 0;
        // Flywheel PID
        public static final double fP = 0;
        public static final double fI = 0;
        public static final double fD = 0;
    
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
        public static final double kP = 0.055773;
        public static final double kI = 0;
        public static final double kD = 0;
        // Trapezoidal motion profile constraints
        public static final double maxA = 20; // acceleration
        public static final double maxV = 40; // velocity
    }

    public static final class HopperConstants {
        public static final double SUCK_SPEED = 0.35;
        public static final double ELEVATOR_SPEED = 0.5;
        public static final double MAX_SPEED = 0.8;
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