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

    //**************** INITIAL ROBOT STATE ****************//
    public static final double STARTING_X = 0,
                               STARTING_Y = 0, 
                               STARTING_HEADING = 0;
    
    //***************** GEAR RATIOS ***********************//
    public static final double LEFT_ENCODER_GEAR_RATIO = 13.7;
    public static final double RIGHT_ENCODER_GEAR_RATIO = 13.7;

    //********************** POINTS ***********************//
    public static final Point POINT_ALPHA = new Point(0, 0);
    public static final Point POINT_BETA  = new Point(0, 0);
    public static final Point POINT_GAMMA  = new Point(0, 0);

}
