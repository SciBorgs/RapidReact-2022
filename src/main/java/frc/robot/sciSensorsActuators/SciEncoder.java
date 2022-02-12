package frc.robot.sciSensorsActuators;

import com.revrobotics.RelativeEncoder;

public class SciEncoder {

    private final double gearRatio; 
    private final double wheelCircumference;
    public RelativeEncoder[] encoders;
    private boolean inverted;

    // public SciEncoder(int channelA, int channelB, double gearRatio, double wheelCircumference) {
    //     super(channelA, channelB);
    //     this.gearRatio = gearRatio;
    //     this.wheelCircumference = wheelCircumference;
    // }

    public SciEncoder(double gearRatio, double wheelCircumference, RelativeEncoder... encoders) {
        this.encoders = encoders;
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;

        this.inverted = false;

        for (RelativeEncoder encoder : this.encoders) {
            encoder.setPosition(0);
        }
    }

    public int get() {
        double val = 0;
        for (RelativeEncoder encoder : this.encoders)
            val += encoder.getPosition() * gearRatio;
        return (int) val * (inverted ? -1 : 1);
    }

    public double getRate() {
        double val = 0;
        for (RelativeEncoder encoder : this.encoders)
            val += encoder.getVelocity() * gearRatio;
        return (int) val * (inverted ? -1 : 1);
    }

    // not sure if this is proper, someone fact check 
    public double getDistance() {
        return get() * wheelCircumference;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void setPosition(double d) {
        for (RelativeEncoder encoder : this.encoders)
            encoder.setPosition(d);
    }
}
