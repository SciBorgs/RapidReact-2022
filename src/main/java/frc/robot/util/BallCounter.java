package frc.robot.util;


public interface BallCounter {
    // due to lack of internal color sensors,
    // we assume that we start with 1 preloaded ball
    AtomicInteger count = new AtomicInteger(1); // ball count starts at 1

    default void incrementBallCount() {
        count.incrementAndGet();
    }
    default void decrementBallCount() {
        count.decrementAndGet();
    }

    default int getBallCount() {
        return count.get();
    }
}
