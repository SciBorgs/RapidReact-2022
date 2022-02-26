package frc.robot.subsystems;

public class DummySubsystem {
    public DummySubsystem() {
        this.number = -1.0;
        this.a = this.b = 0.0;
    }

    // Testing getting and setting

    private double number;

    public double getNumber() {
        return this.number;
    }

    public String getWord() {
        return "sciborgs";
    }

    public void setNumber(double val) {
        this.number = val;
    }

    public double get4() {
        return 4.0;
    }

    // calculator

    private double a, b;

    public void setAddend1(double a) { this.a = a; }
    public void setAddend2(double b) { this.b = b; }
    public double getSum() { return a + b; }
}
