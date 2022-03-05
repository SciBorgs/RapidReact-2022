package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

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

    public static Point add(Point... ps) {
        double x = 0;
        double y = 0;
        for (Point p : ps) {
            x += p.x;
            y += p.y;
        }
        return new Point(x, y);
    }

    public static Point subtract(Point p1, Point p2) {
        return displacementVector(p2, p1);
    }

    public static Point scale(Point p, double k) {
        return new Point(k * p.x, k * p.y);
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

    public static double length(List<Point> curve, int skip) {
        double length = 0;
        int n = curve.size();
        Point prevPoint = curve.get(0);
        for (int i = skip; i < n; i += skip) {
            Point currPoint = curve.get(i);
            length += distance(prevPoint, currPoint);
            prevPoint = currPoint;
        }
        length += distance(prevPoint, curve.get(n - 1));
        return length;
    }

    // Generates a path to be used for testing discrete path following
    public static List<Point> generateRandomPath(int n, double x1, double y1, double x2, double y2) {
        List<Point> points = new ArrayList<>();
        for (int i = 0; i < n; i++)
            points.add(generateRandomPoint(x1, y1, x2, y2));
        return points;
    }

    public static Point generateRandomPoint(double x1, double y1, double x2, double y2) {
        return new Point(map(Math.random(), 0.0, 1.0, x1, x2),
                         map(Math.random(), 0.0, 1.0, y1, y2));
    }

    public static List<Point> generateSinePath(double length, double amplitude, double frequency) {
        List<Point> points = new ArrayList<>();
        for (double t = 0; t < length; t+=0.05) {
            points.add(new Point(t, amplitude * Math.sin(frequency * t)));
        }
        return points;
    }
}
