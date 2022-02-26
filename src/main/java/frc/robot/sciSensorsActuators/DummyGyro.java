package frc.robot.sciSensorsActuators;

import frc.robot.Constants;

// For simulation purposes only.
public class DummyGyro {
    public SciEncoder left, right;
    private double offset;

    public DummyGyro(SciEncoder left, SciEncoder right) {
        this.left = left;
        this.right = right;
        this.offset = 0;
        right.setInverted(true);
    }

    public void setAngle(double val) {
        this.offset += val - getAngle();
    }

    public double getAngle() {
        return this.offset + (right.getDistance() - left.getDistance()) / Constants.ROBOT_WIDTH;
    }

    public double getAngularVelocity() {
        return (right.getSpeed() - left.getSpeed()) / Constants.ROBOT_WIDTH / 60 / -10;
    }
}
