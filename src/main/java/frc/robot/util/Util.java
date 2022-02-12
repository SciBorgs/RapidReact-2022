package frc.robot.util;

public class Util {
    public static double normalize(double v) {
        return Math.min(Math.max(-1, v), 1);
    }

    public static double getDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }
}
