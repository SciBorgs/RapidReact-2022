package frc.robot.controllers;

import frc.robot.Robot;

public class Following {
    public static final double K_TX = 1.;
    public static final double K_TA = 1.;

    public void follow(double dAngle) {
        double tx = Robot.limelightSubsystem.getTableData("tx");
        double ta = Robot.limelightSubsystem.getTableData("ta");

        double angle = tx * K_TX;
        double distance = ta * K_TA;

        Robot.driveSubsystem.setSpeedForwardAngle(distance, angle);
    }
}
