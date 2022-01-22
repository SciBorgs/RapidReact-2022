package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;

public class Following {
    // public static final double K_TX = 1.;
    public static final double K_TA = 1.;
    
    public static final double TX_P = 0.035;

    public PID txPID;

    public Following() {
        this.txPID = new PID(TX_P, 0, 0);
    }

    public void follow() {
        double tx = Robot.limelightSubsystem.getTableData("tx");
        double ta = Robot.limelightSubsystem.getTableData("ta");

        double distance = ta * K_TA;

        Robot.driveSubsystem.setSpeedForwardAngle(distance, txPID.getOutput(0, tx));
    }
}
