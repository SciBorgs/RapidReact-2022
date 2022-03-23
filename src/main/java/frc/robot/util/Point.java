package frc.robot.util;

public class Point {
    public final double x, y;

    public Point(double[] v) {
        this.x = v[0];
        this.y = v[1];
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return String.format("(%f, %f)", this.x, this.y);
    }

    public double[] toArray() {
        return new double[] {this.x, this.y};
    }
}
