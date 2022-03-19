package frc.robot;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Path;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Use this class for storing paths, points, etc. in the auto sequence.
 */
public class AutoProfile {
    public static final double SHOOTING_RADIUS_NEAR = 1.25;
    public static final double SHOOTING_RADIUS_FAR  = 3.3;

    public static final List<Point> PATH_TEST = Util.generateSinePath(4.0, 0.45, 1.35, 0.05);
    public static final Point POINT_TEST = new Point(0, 0);

    public static final HashMap<String, Path> AUTON_PATHS = new HashMap<>();

    public boolean initialize() {
        
    }

    public Command getCommand() {
        return null;
    }
}
