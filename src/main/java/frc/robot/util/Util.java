package frc.robot.util;

public class Util {
    public static double normalize(double v) {
        return Math.min(Math.max(-1, v), 1);
    }

    public static double getDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    public static Point unitVector(double theta) {
        return new Point(Math.cos(theta), Math.sin(theta));
    }

    public static Point displacementVector(Point current, Point desired) {
        return new Point(desired.x - current.x, desired.y - current.y);
    }

    public static double dot(Point a, Point b) {
        return a.x * b.x + a.y * b.y;
    }
}
