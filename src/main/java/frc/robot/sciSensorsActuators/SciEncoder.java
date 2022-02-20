package frc.robot.sciSensorsActuators;

import com.revrobotics.RelativeEncoder;

public class SciEncoder {
    private final double gearRatio; 
    private final double wheelCircumference;
    private RelativeEncoder[] encoders;
    private boolean[] inverted;

    //conversion between whatever unit it gives and meters
    // gotta love hardcoding!
    private static final double ROYS_CONSTANT = 113.08662 * 4.696466 * 1.377619 * 0.5;

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
        return val * gearRatio * wheelCircumference / this.encoders.length / ROYS_CONSTANT;
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
                  * gearRatio * wheelCircumference
                  * (this.inverted[i] ? -1.0 : 1.0)
            );
        }
        sb.append("\nTotal : " + this.getDistance());
        return sb.toString();
    }
}
