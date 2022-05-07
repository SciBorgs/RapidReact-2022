package frc.robot.sciSensorsActuators;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciAbsoluteEncoder extends DutyCycleEncoder {
    private double factor;
    public double offset;

    public SciAbsoluteEncoder(int port, double gearRatio, double offset) {
        super(port);
        this.factor = gearRatio * 360;
        this.offset = offset;
    }

    public SciAbsoluteEncoder(int port, double gearRatio) {
        this(port, gearRatio, 0.0);
    }

    public double getAngle() {
        return getAbsolutePosition() * factor - offset;
    }
}
