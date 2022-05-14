package frc.robot.util;

public class DelayedPrinter {
    private long prevPrint;
    private long n;

    public DelayedPrinter(long n) {
        this.prevPrint = System.currentTimeMillis();
        this.n = n;
    }

    public void print(String str) {
        long currTime = System.currentTimeMillis();
        if (currTime - this.prevPrint > n) {
            this.prevPrint = currTime;
            System.out.println(str);
        }
    }
}
