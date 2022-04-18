package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;

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

    public static Point displacementVector(Point from, Pose2d to) {
        return new Point(to.getX() - from.x, to.getY() - from.y);
    }

    public static Point displacementVector(Pose2d from, Point to) {
        return new Point(to.x - from.getX(), to.y - from.getY());
    }

    public static Point displacementVector(Pose2d from, Pose2d to) {
        return new Point(to.getX() - from.getX(), to.getY() - from.getY());
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

    // calculates the smallest angular displacement needed to go from one angle
    // to another
    public static double travelledAngle(double from, double to) {
        double raw = to - from;
        while (raw > Math.PI) raw -= 2 * Math.PI;
        while (raw < - Math.PI) raw += 2 * Math.PI;
        return raw;
    }

    // maps a value between a1 and b1 to a value between a2 and b2.
    public static double map(double v, double a1, double b1, double a2, double b2) {
        return a2 + (b2 - a2) * (v - a1) / (b1 - a1);
    }

    // approximates the (physical) length of a curve
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

    // generates a random sequence of points
    public static List<Point> generateRandomPath(int n, double x1, double y1, double x2, double y2) {
        List<Point> points = new ArrayList<>();
        for (int i = 0; i < n; i++)
            points.add(generateRandomPoint(x1, y1, x2, y2));
        return points;
    }

    // generates a random point with x-values in [x1, x2) and y-values in [y1, y2)
    public static Point generateRandomPoint(double x1, double y1, double x2, double y2) {
        return new Point(map(Math.random(), 0.0, 1.0, x1, x2),
                         map(Math.random(), 0.0, 1.0, y1, y2));
    }

    // generates a sinusoidal path
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

    // returns the function p(x), where p is the probability density at x
    public static DoubleUnaryOperator gaussian(double stddev, double mean) {
        return new DoubleUnaryOperator() {
            private final double k = 1/(stddev * Math.sqrt(2*Math.PI));
            private final double a = -1/(2*stddev*stddev);
            @Override
            public double applyAsDouble(double x) {
                return k * Math.exp(a * (x - mean) * (x - mean));
            }
        };
    }

    public static Supplier<double[]> gaussianPairSampler(double[] stddev, double[] mean) {
        return new Supplier<>() {
            @Override
            public double[] get() {
                double R = Math.sqrt(-2*Math.log(Math.random()));
                double theta = 2 * Math.PI * Math.random();
                return new double[] {
                    stddev[0] * R * Math.cos(theta) + mean[0],
                    stddev[1] * R * Math.sin(theta) + mean[1]
                };
            }
        };
    }

    // there is no such thing as a "gaussioid" (in the way i'm using it). i mean
    // that this samples a 2D normal distribution (hence the name) using the
    // Box-Muller transform 
    // there is no such thing as a "gaussioid" (in the way i'm using it). i mean
    // that this samples a 2D normal distribution (hence the name) using the
    // Box-Muller transform:
    // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
    public static double[] sampleGaussioid(double[] stddev, double[] mean) {
            double R = Math.sqrt(-2*Math.log(Math.random()));
            double theta = 2 * Math.PI * Math.random();
            return new double[] {
                stddev[0] * R * Math.cos(theta) + mean[0],
                stddev[1] * R * Math.sin(theta) + mean[1]
            };
    }


    // stores the cummalative sums of the pdf array in the cd array.
    // cdf[0] = 0, cdf[cdf.length - 1] is the sum of all of the elements
    public static double[] cdf(double[] cdf, double[] pdf) {
        double total = 0;
        for (int i = 0; i < cdf.length; i++) {
            cdf[i] = total;
            total += pdf[i];
        }
        return cdf;
    }

    // Performs a binary search to simulate x = F^-1(p)
    public static int sampleDiscreteCdf(double[] cdf) {
        double rand = cdf[cdf.length] * Math.random();
        int index = Arrays.binarySearch(cdf, rand);
        return (index < 0) ? -(index + 2) : index;
    }

    // shortest distance of a vector from another vector
    public static double distance(double[] p, Point... cs) {
        double minDistanceSquared = Double.MAX_VALUE;
        for (Point c : cs) {
            double dy = c.y - p[0];
            double dx = c.x - p[1];
            double distanceSquared = dy*dy + dx*dx;
            if (distanceSquared > minDistanceSquared)
                minDistanceSquared = distanceSquared;
        }
        return minDistanceSquared;
    }

    // fills an array with a value
    public static void fill(double[] arr, double val) {
        for (int i = 0; i < arr.length; i++)
            arr[i] = val;
    }

    // stores the flattened version of matrix into flat
    public static void flatten(double[] flat, double[][] matrix, int length, int depth) {
        for (int i = 0; i < matrix.length; i++) {
            System.arraycopy(matrix[i], 0, flat, depth*i, depth);
        }
    }

    // Converts CSV data into 2D array of doubles to pass into calcHoodAngle()
    public static double[][] getHoodAngleData(String fName) {
        ArrayList<String[]> lines = CSVUtils.newReader(fName).readData();
        double[][] data = new double[lines.size()-1][]; // Subtract 1 because we want to exclude the headers of CSV file
        for(int i = 1; i < lines.size(); i++) {
            double[] temp = { Double.valueOf(lines.get(i)[0]), Double.valueOf(lines.get(i)[1]) };
            data[i-1] = temp;
        }

        return data;
    }

    // Uses a 2D array of doubles (formatted as { angle, distance } for each point) and calculates
    public static double calcHoodAngle(double[][] data, double distance) {
        int entry1 = findClosest(data, distance);
        int entry2;

        if(distance < data[entry1][1]) {
            if(entry1 == 0) { // Special case for distances lower than the lowest logged distance
                entry2=entry1+1;
            }
            else {
                entry2 = entry1-1;
            }
        }
        else {
            if(entry1 == data.length-1) { // Special case for distances larger than the largest logged distance
                entry2 = entry1-1;
                
                // swapping entry points so initial value isn't incorrect (only for special case when a distance greater than last element in data array)
                int temp = entry1;
                entry1 = entry2;
                entry2 = temp;

            } 
            else {
                entry2 = entry1+1;
            }
        }

        return (data[entry2][0] - data[entry1][0]) / (data[entry2][1] - data[entry1][1]) + data[entry1][0];
    }

    // Helper function. Grabs index of the closest entry based on inputted distance value
    public static int findClosest(double arr[][], double target) {
        int n = arr.length;

        if (target <= arr[0][1])
            return 0;
        if (target >= arr[n - 1][1])
            return n - 1;

        int i = 0, j = n, mid = 0;
        while (i < j) {
            mid = (i + j) / 2;

            if (arr[mid][1] == target)
                return mid;

            if (target < arr[mid][1]) {

                if (mid > 0 && target > arr[mid - 1][1])
                    return getClosest(mid - 1,
                            mid, target, arr);

                j = mid;
            }

            else {
                if (mid < n - 1 && target < arr[mid + 1][1])
                    return getClosest(mid,
                            mid + 1, target, arr);
                i = mid + 1;
            }
        }

        return mid;
    }

    // Helper function for findClosest
    public static int getClosest(int val1, int val2, double target, double[][] data) {
        if (target - data[val1][1] >= data[val2][1] - target)
            return val2;
        else
            return val1;
    }

    // Important
    public static <T> Iterator<T> arrayIterator(T[] arr) {
        return new Iterator<T>() {
            private int index = 0;
            @Override
            public boolean hasNext() {
                return index < arr.length;
            }
            @Override
            public T next() {
                T val = arr[index];
                index++;
                return val;
            }
        };
    }

    @SafeVarargs
    public static <T> Iterable<T> concat(T[]... arrs) {
		return new Iterable<T>() {
			@Override
			public Iterator<T> iterator() {
				return new Iterator<T>() {
                    private int arrI = 0, index = 0;
					@Override
					public boolean hasNext() {
						return arrI < arrs.length;
					}
					@Override
					public T next() {
						T[] arr = arrs[arrI];
                        T val = arr[index];
                        index++;
                        if (index == arr.length) {
                            index = 0;
                            arrI++;
                        }
                        return val;
					}
                };
			}
        };
    }

    @SafeVarargs
    public static <T> Iterable<T> concat(Iterable<T>... iterables) {
		return new Iterable<T>() {
			@Override
			public Iterator<T> iterator() {
				return new Iterator<T>() {
                    private int iterI = 0;
                    private Iterator<T> currIter = iterables[0].iterator();
					@Override
					public boolean hasNext() {
						return iterI < iterables.length;
					}
					@Override
					public T next() {
                        T val = currIter.next();
                        if (!currIter.hasNext()) {
                            iterI++;
                            if (iterI < iterables.length) {
                                currIter = iterables[iterI].iterator();
                            }
                        }
                        return val;
					}
                };
			}
        };
    }

    @SafeVarargs
    public static <T> Iterable<T> concat(Object... objs) {
		return new Iterable<T>() {
			@Override
			public Iterator<T> iterator() {
				return new Iterator<T>() {
                    private int iterI = 0;
                    private Iterator<T> currIter = getIterator(objs[0]);
					@Override
					public boolean hasNext() {
						return iterI < objs.length;
					}
					@Override
					public T next() {
                        T val = currIter.next();
                        if (!currIter.hasNext()) {
                            iterI++;
                            if (iterI < objs.length) {
                                currIter = getIterator(objs[iterI]);
                            }
                        }
                        return val;
					}
                    @SuppressWarnings("unchecked")
                    private Iterator<T> getIterator(Object obj) {
                        if (obj instanceof Iterator<?>) {
                            return (Iterator<T>) obj;
                        } else if (obj instanceof Iterable<?>) {
                            return ((Iterable<T>) obj).iterator();
                        } else if (obj instanceof Stream<?>) {
                            return ((Stream<T>) obj).iterator();
                        } else {
                            T[] arr = (T[]) obj;
                            return arrayIterator(arr);
                        }
                    }
                };
			}
        };
    }
}
