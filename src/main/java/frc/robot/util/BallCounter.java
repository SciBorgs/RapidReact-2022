package frc.robot.util;

import java.util.concurrent.atomic.AtomicInteger;

public interface BallCounter {
    // due to lack of internal color sensors,
    // we assume that we start with 1 preloaded ball
    // atomic integer is a hack to have an int that can be incremented and decremented in an interface
    AtomicInteger count = new AtomicInteger(1); // ball count starts at 1

    default void increment() {
        count.incrementAndGet();
    }
    default void decrement() {
        count.decrementAndGet();
    }

    default int get() {
        return count.get();
    }
}
