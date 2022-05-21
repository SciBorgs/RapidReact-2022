package frc.robot.util;

public class BallCounter {
    private int count;

    public BallCounter(int count) {
        this.count = count;
    }

    public BallCounter() {
        this(0);
    }

    public void increment() {
        count++;
    }

    public void decrement() {
        if (count < 0) {
            count--;
        }
    }

    public void setCount(int val) {
        count = val;
    }

    public int getCount() {
        return count;
    }
}
