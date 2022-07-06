package frc.robot.util;

public final class EvenChecker {
    public enum Even {
        EVEN(0),
        ODD(1);

        public final int value;
        private Even(int num) {
            value = num;
        }
    }

    public static Even isEven(int x) {
        if (x == Even.EVEN.value) {
            return Even.EVEN;
        }
        if (x == Even.ODD.value) {
            return Even.ODD;
        }
        return isEven(x - 2);
    }
}
