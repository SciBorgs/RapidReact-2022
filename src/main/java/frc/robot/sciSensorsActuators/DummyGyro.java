package frc.robot.sciSensorsActuators;

import frc.robot.Constants;

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
}
