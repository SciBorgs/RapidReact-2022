package frc.robot.util;

public class Util {
    public static double normalize(double v) {
        return Math.min(Math.max(-1, v), 1);
    }

    public static double getDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    public static double getDistance(Point p, Ring c) {
        double distance = getDistance(p, c.center);
        return Math.abs(distance - c.radius);
    }

    public static Point unitVector(double theta) {
        return new Point(Math.cos(theta), Math.sin(theta));
    }

    public static Point displacementVector(Point from, Point to) {
        return new Point(from.x - to.x, to.y - from.y);
    }

    public static double dot(Point a, Point b) {
        return a.x * b.x + a.y * b.y;
    }
}
