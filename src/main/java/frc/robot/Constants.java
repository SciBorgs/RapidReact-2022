// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.controllers.PathSmoother;
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
    
    //**************** ROBOT DIMENSIONS *******************//
    public static final double ROBOT_WIDTH = 0.5588; 
    public static final double WHEEL_CIRCUMFERENCE = 0.4787787204; //..... :)

    //**************** INITIAL ROBOT STATE ****************//
    public static final Point  STARTING_POINT = new Point(0, 0);
    public static final double STARTING_HEADING = 0;
    
    //***************** GEAR RATIOS ***********************//
    public static final double WHEEL_ENCODER_GEAR_RATIO = 13.7;

    //********************** FIELD ***********************//
        // Note: The origin is the center of the hub.
    public static final Point POINT_HUB  = new Point(0, 0);
    public static final Point POINT_BETA = new Point(0, 0);
    public static final Point POINT_TEST = new Point(-2, 0);

        // Adjust radii here for desired shooting/searching distances.
    public static final double SHOOTING_RADIUS_NEAR = 1.25;
    public static final double SHOOTING_RADIUS_FAR  = 3.3;

    public static final List<Point> PATH_TEST_RAW = new ArrayList<>();
    static {
        PATH_TEST_RAW.add(STARTING_POINT);
        PATH_TEST_RAW.add(new Point(0.5, 0.2));
        PATH_TEST_RAW.add(new Point(0, -0.5));
        PATH_TEST_RAW.add(new Point(-0.5, 0.0));
        // PATH_TEST_RAW.add(new Point(8, 0));
    }
    public static final List<Point> PATH_TEST = new PathSmoother(PATH_TEST_RAW, 20).getFinalPath();
    
    //***************** GEAR RATIOS ***********************//
    public static final double LEFT_ENCODER_GEAR_RATIO = 13.7;
    public static final double RIGHT_ENCODER_GEAR_RATIO = 13.7;
    public static final double SMALL_TURRET_GEAR_RATIO = RIGHT_ENCODER_GEAR_RATIO / 10;
    public static final double TOTAL_HOOD_GEAR_RATIO = 1/230;
}
