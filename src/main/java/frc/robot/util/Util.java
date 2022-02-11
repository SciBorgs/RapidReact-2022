package frc.robot.util;

public class Util {
    public static double normalize(double v) {
        return Math.min(Math.max(-1, v), 1);
    }
}
