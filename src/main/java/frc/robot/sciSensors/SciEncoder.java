package frc.robot.sciSensors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

public class SciEncoder {
    private RelativeEncoder[] encoders;
    private boolean[] inverted;

    private static final double ROYS_CONSTANT = 365.831868022;
    private double factor;

    public SciEncoder(double gearRatio, double wheelCircumference, SciSpark... sparks) {
        this.factor = gearRatio * wheelCircumference / ROYS_CONSTANT;

        this.encoders = new RelativeEncoder[sparks.length];
        for (int i = 0; i < sparks.length; i++) {
            encoders[i] = sparks[i].getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); // TODO fix
            encoders[i].setPosition(0);
        }
        this.inverted = new boolean[this.encoders.length];
    }

    // allows us to pass in several encoders to be averaged (i.e. w/ drivetrain encoders)
    public SciEncoder(double gearRatio, double wheelCircumference, RelativeEncoder... encoders) {
        this.factor = gearRatio * wheelCircumference / ROYS_CONSTANT;

        this.encoders = encoders;
        this.inverted = new boolean[this.encoders.length];

        for (RelativeEncoder encoder : this.encoders)
            encoder.setPosition(0);
    }

    // encoder units
    private double getRawDistance() {
        double val = 0;
        for (int i = 0; i < this.encoders.length; i++) {
            RelativeEncoder encoder = this.encoders[i];
            val += encoder.getPosition() * (this.inverted[i] ? -1.0 : 1.0);
        }
        return val / this.encoders.length;
    }

    // encoder units
    private double getRawSpeed() {
        double val = 0;
        for (int i = 0; i < this.encoders.length; i++) {
            RelativeEncoder encoder = this.encoders[i];
            val += encoder.getVelocity() * (this.inverted[i] ? -1.0 : 1.0);
        }
        return val / this.encoders.length;
    }
    public double getDistance() {
        return this.getRawDistance() * this.factor;
    }

    public void reset() {
        for (RelativeEncoder encoder : this.encoders) {
            encoder.setPosition(0);
        }
    }

    public double getSpeed() {
        return this.getRawSpeed() * this.factor;
    }

    public int getNumEncoders() {
        return this.encoders.length;
    }

    // set individual encoders to be read in reverse
    public void setInverted(boolean... inverted) {
        if (inverted.length != this.inverted.length) throw new IllegalArgumentException();
        System.arraycopy(inverted, 0, this.inverted, 0, inverted.length);
    }

    public void setDistance(double d) {
        for (RelativeEncoder encoder : this.encoders)
            encoder.setPosition(d);
    }

    public String getInfoString() {
        StringBuilder sb = new StringBuilder();
        sb.append('\n');
        for (int i = 0; i < this.encoders.length; i++) {
            sb.append("\nEncoder " + i + " : " 
                + this.encoders[i].getPosition() 
                  * (this.inverted[i] ? -1.0 : 1.0)
                  / ROYS_CONSTANT
            );
        }
        sb.append("\nTotal : " + this.getDistance());
        return sb.toString();
    }
}
