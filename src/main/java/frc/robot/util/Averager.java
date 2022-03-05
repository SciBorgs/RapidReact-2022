package frc.robot.util;

public class Averager {

    private double weight;
    private double average;

    public Averager (double weight) {
        this.weight = weight;
    }

    public double getAverage(double value) {
        average = weight * value + (1 - weight) * average;
        return average;
    }

}
