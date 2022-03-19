package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Represents a timeless path paramaterized by length.
 */
public class Path extends ArrayList<Point> {
    public final double length, speed;
    private final int n;

    protected Path(List<Point> points, double length) {
        super(points);
        this.length = length;
        this.n = points.size();
        this.speed = length / n;
    }
}
