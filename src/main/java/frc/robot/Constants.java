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
    //**************** ROBOT DIMENSIONS *******************//
    public static final double ROBOT_WIDTH = 0.5588; 
    public static final double WHEEL_CIRCUMFERENCE = 0.4787787204; //..... :)
    
    //***************** GEAR RATIOS ***********************//
    public static final double WHEEL_ENCODER_GEAR_RATIO = 13.7;

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
}
