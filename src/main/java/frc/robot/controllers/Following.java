package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Following {
    // public static final double K_TX = 1.;
    public static final double K_TA = 1.;
    
    public static final double TX_P = 0.035;

    public static PID txPID;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");
        // double ta = Robot.limelightSubsystem.getTableData(table, "ta");

        // double distance = ta * K_TA;
        // double forward = distance;
        double forward = 0.5;

        Robot.driveSubsystem.setSpeedForwardAngle(distance, txPID.getOutput(0, tx));
    }
}
