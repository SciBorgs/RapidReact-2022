package frc.robot.util;

public class Util {
    public static double normalize(double v) {
        return Math.min(Math.max(-1, v), 1);
    }

    public static double normalize(double v, double absmax) {
        return Math.min(Math.max(-absmax, v), absmax);
    }

    public static double distanceSquared(Point a, Point b) {
        return Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2);
    }

    public static double distance(Point a, Point b) {
        return Math.sqrt(distanceSquared(a, b));
    }

    public static double norm(Point v) {
        return Math.sqrt(v.x * v.x + v.y * v.y);
    }
    
    public static Point unitVector(double theta) {
        return new Point(Math.cos(theta), Math.sin(theta));
    }

    public static Point displacementVector(Point from, Point to) {
        return new Point(to.x - from.x, to.y - from.y);
    }

    public static double angleToPoint(Point p) {
        return Math.atan2(p.y, p.x);
    }

    public static double dot(Point a, Point b) {
        return a.x * b.x + a.y * b.y;
    }

    public static double travelledAngle(double from, double to) {
        double raw = to - from;
        while (raw > Math.PI) raw -= 2 * Math.PI;
        while (raw < - Math.PI) raw += 2 * Math.PI;
        return raw;
    }

    public static double map(double v, double a1, double b1, double a2, double b2) {
        return a2 + (b2 - a2) * (v - a1) / (b1 - a1);
    }

    // Generates a path to be used for testing discrete path following
    public static Path generateRandomPath(int n, double x1, double y1, double x2, double y2) {
        Point[] points = new Point[n];
        for (int i = 0; i < n; i++) {
            points[i] = new Point(
                map(Math.random(), 0.0, 1.0, x1, x2),
                map(Math.random(), 0.0, 1.0, y1, y2)
            );
        }
        return new Path(points);
    }
}
