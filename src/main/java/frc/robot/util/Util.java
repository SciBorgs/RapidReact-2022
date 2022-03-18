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

    public static List<Point> generateSinePath(double length, double amplitude, double frequency, double step) {
        List<Point> points = new ArrayList<>();
        for (double t = 0; t < length; t+=step) {
            points.add(new Point(t, amplitude * Math.sin(frequency * t)));
        }
        return points;
    }

    /**
     * Determines the heading of the robot when it reacheds a point along a certain path.
     * @param path the path that the robot is following
     * @param interceptionPoint the point at which to find the heading
     * @param predictDistanceSquared the acceptable distance on a from the point, squared.
     * @param step the interval on the curve on which to approximate the derivative
     * @return the expected heading of the robot
     * @throws IllegalArgumentException if the point is too far away from the curve
     */
    public static double interceptionAngle(List<Point> path, Point interceptionPoint, double predictDistanceSquared, int step) {
        int searchIndex = 0;
        int n = path.size();
        boolean found = false;
        while (++searchIndex < n && !found) {
            Point near = path.get(searchIndex);
            if (distanceSquared(interceptionPoint, near) < predictDistanceSquared)
                found = true;
        }

        if (!found) throw new IllegalArgumentException("Point not found in the path!");
        Point p1 = path.get(Math.max(0, searchIndex - step/2)); // derivative measurement is already delayed
        Point p2 = path.get(Math.min(searchIndex + step, n - 1));
        return Math.atan2(p2.y - p1.y, p2.x - p1.x);
    }

    /**
     * Returns a 'parameterization' of a curve where input is proportional to 
     * arc length.
     * @param curve a continuous sequence of points
     * @param lengthStep the resolution to use
     * @return a reparamaterization of the given curve
     */
    public static Path reparameterize(List<Point> curve, double lengthStep) {
        double arclength = length(curve, 1);
        double forbidden = Math.pow(lengthStep * 0.9, 2);
        List<Point> reparameterized = new ArrayList<Point>();

        Point prevPoint = curve.get(0);
        double s = 0;
        int i = 1;
        int n = 0;
        while (s < arclength && i < curve.size()) {
            Point currPoint = curve.get(i);
            double ds = distance(prevPoint, currPoint);
            // Number of points to linearly interpolate. Preferrably 0 or 1.
            int pointsToEstimate = (int) ((s + ds) / lengthStep) - (int) (s / lengthStep);
            for (int j = 1; j <= pointsToEstimate; j++) {
                double k = lengthStep * j / ds;
                Point interpolated = add(scale(prevPoint, 1-k), scale(currPoint, k));
                /* With a large length step and low initial curve resolution,
                   linearly interpolated points will stray from the actual curve.
                   This has a tendency to cause "spikes" in the final curve,
                   which may be very inconvenient if you want to find the slope
                   at some point on the resulting curve.

                   Instead of removing such spikes, which would sacrifice the
                   correspondence between indices and arc length, we just replace
                   a spike with the next point on the curve.

                   Just don't make your index lookahead 1.                          */
                if (j == 1 && n > 0) {
                    Point prev = reparameterized.get(n - 1);
                    if (distanceSquared(prev, interpolated) < forbidden)
                        reparameterized.set(n - 1, interpolated);
                }
                reparameterized.add(interpolated);
                n++;
            }
            prevPoint = currPoint;
            i++;
            s += ds;
        }

        return new Path(reparameterized, arclength);
    }
}
