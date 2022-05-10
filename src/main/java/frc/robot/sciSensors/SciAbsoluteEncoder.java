package frc.robot.sciSensors;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SciAbsoluteEncoder extends DutyCycleEncoder {
    private double ratio;
    public double offset;

    public SciAbsoluteEncoder(int port, double ratio, double offset) {
        super(port);
        this.ratio = ratio;
        this.offset = offset;
    }

    public SciAbsoluteEncoder(int port, double ratio) {
        this(port, ratio, 0.0);
    }

    // degrees
    public double getAngle() {
        return Units.rotationsToDegrees(getAbsolutePosition() * ratio) - offset;
    }
}
