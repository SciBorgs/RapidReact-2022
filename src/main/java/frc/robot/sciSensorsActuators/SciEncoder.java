package frc.robot.sciSensorsActuators;

import com.revrobotics.RelativeEncoder;

public class SciEncoder {

    private final double gearRatio; 
    private final double wheelCircumference;
    public RelativeEncoder encoder;
    private boolean inverted;

    // public SciEncoder(int channelA, int channelB, double gearRatio, double wheelCircumference) {
    //     super(channelA, channelB);
    //     this.gearRatio = gearRatio;
    //     this.wheelCircumference = wheelCircumference;
    // }

    public SciEncoder(RelativeEncoder encoder, double gearRatio, double wheelCircumference) {
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;

        this.inverted = false;

        this.encoder.setPosition(0);
    }

    public int get() {
        return (int) (encoder.getPosition() * gearRatio * (inverted ? -1 : 1));
    }

    public double getRate() {
        return encoder.getVelocity() * gearRatio * (inverted ? -1 : 1);
    }

    // not sure if this is proper, someone fact check 
    public double getDistance() {
        return get() * wheelCircumference;
        
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
}