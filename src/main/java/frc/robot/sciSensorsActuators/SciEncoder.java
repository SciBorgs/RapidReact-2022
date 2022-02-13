package frc.robot.sciSensorsActuators;

import com.revrobotics.RelativeEncoder;

public class SciEncoder {
    private final double gearRatio; 
    private final double wheelCircumference;
    private RelativeEncoder[] encoders;
    private boolean[] inverted;

    // allows us to pass in several encoders to be averaged (i.e. w/ drivetrain encoders)
    public SciEncoder(double gearRatio, double wheelCircumference, RelativeEncoder... encoders) {
        this.encoders = encoders;
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;

        this.inverted = new boolean[this.encoders.length];

        for (RelativeEncoder encoder : this.encoders)
            encoder.setPosition(0);
    }

    public int get() {
        double val = 0;
        for (int i = 0; i < this.encoders.length; i++) {
            RelativeEncoder encoder = this.encoders[i];
            val += encoder.getPosition() * (this.inverted[i] ? -1.0 : 1.0);
        }
        return (int) (val * gearRatio / this.encoders.length);
    }

    public double getRate() {
        double val = 0;
        for (int i = 0; i < this.encoders.length; i++) {
            RelativeEncoder encoder = this.encoders[i];
            val += encoder.getVelocity() * (this.inverted[i] ? -1.0 : 1.0);
        }
        return val * gearRatio * wheelCircumference / this.encoders.length;
    }

    public double getDistance() {
        double val = 0;
        for (int i = 0; i < this.encoders.length; i++) {
            RelativeEncoder encoder = this.encoders[i];
            val += encoder.getPosition() * (this.inverted[i] ? -1.0 : 1.0);
        }
        return val * gearRatio * wheelCircumference / this.encoders.length;
    }

    public int getNumEncoders() {
        return this.encoders.length;
    }

    // set individual encoders to be read in reverse
    public void setInverted(boolean... inverted) {
        if (inverted.length != this.inverted.length) throw new IllegalArgumentException();
        System.arraycopy(inverted, 0, this.inverted, 0, inverted.length);
    }

    public void setPosition(double d) {
        for (RelativeEncoder encoder : this.encoders)
            encoder.setPosition(d);
    }
}
