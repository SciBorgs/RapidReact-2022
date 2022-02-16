package frc.robot.util;

import java.util.Iterator;

public class Path {
    public final Point[] points;

    public Path(Point... points) {
        this.points = new Point[points.length];
        System.arraycopy(points, 0, this.points, 0, points.length);
    }
    
    public Iterator<Point> openIterator() {
        return new Iterator<Point>() {
            private int i = 0;

			@Override
			public boolean hasNext() {
				return i < points.length;
			}

			@Override
			public Point next() {
                Point p = points[i];
                i++;
                return p;
			}  
        };
    }

    public Iterator<Point> closedIterator() {
        return new Iterator<Point>() {
            private int i = 0;

			@Override
			public boolean hasNext() {
				return true;
			}

			@Override
			public Point next() {
                Point p = points[i];
                i = (i + 1) % points.length;
                return p;
			}  
        };
    }
}
