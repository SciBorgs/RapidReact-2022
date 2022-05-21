package frc.robot.util;

public interface Counter {
    // due to lack of internal color sensors,
    // we assume that we start with 1 preloaded ball
    BallCounter count = new BallCounter(1); // ball count starts at 1

    void increment();
    void decrement();
    int get();
}
