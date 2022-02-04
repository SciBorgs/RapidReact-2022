package frc.robot.controllers;

public class PointFollowing {
    public static Waypoint[] points = { };
}

class Waypoint {
    public int x, y;
    public Waypoint(int x, int y) {
        this.x = x;
        this.y = y;
    }
}
