package frc.robot.subsystems;

public class DummySubsystem {
    public DummySubsystem() {
        this.number = -1.0;
    }

    private double number;

    public double getNumber() {
        return this.number;
    }

    public void setNumber(Object val) {
        this.number = (Double) val;
    }

    public double get4() {
        return 4.0;
    }
}
