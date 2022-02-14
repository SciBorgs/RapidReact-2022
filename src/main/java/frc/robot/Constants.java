// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.Point;
import frc.robot.util.Ring;

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
    public static final Point  STARTING_POINT = new Point(0, 0);
    public static final double STARTING_HEADING = 0;
    
    //***************** GEAR RATIOS ***********************//
    public static final double LEFT_ENCODER_GEAR_RATIO = 13.7;
    public static final double RIGHT_ENCODER_GEAR_RATIO = 13.7;

    //********************** POINTS ***********************//
        // Note: The origin is the center of the hub.
    public static final Point POINT_ALPHA = new Point(0, 0);
    public static final Point POINT_BETA  = new Point(0, 0);
    public static final Point POINT_GAMMA = new Point(0, 0);
    public static final Point POINT_TEST  = new Point(2, 0);    

        // Adjust radii here for desired shooting/searching distances.
    public static final Ring RING_ALPHA = new Ring(POINT_ALPHA, 1.25);
    public static final Ring RING_BETA  = new Ring(POINT_BETA, 3.0);
    public static final Ring RING_GAMMA = new Ring(POINT_GAMMA, 3.3);
}
