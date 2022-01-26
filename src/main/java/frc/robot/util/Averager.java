package frc.robot.util;

import frc.robot.Robot;

import java.util.Deque;
import java.util.LinkedList;


public class Averager {

    private final int limit;

    public Averager(int limit) {
        this.limit = limit;
    }

    private Deque<Double> deque = new LinkedList<Double>();
    private double sum = 0;

    public double getAverageTA(double value) {
        deque.addFirst(value);
        sum += value;
        if(deque.size() > limit) {
            sum -= deque.getLast();
            deque.removeLast();
        }
        
        return sum/deque.size();
    }


}
