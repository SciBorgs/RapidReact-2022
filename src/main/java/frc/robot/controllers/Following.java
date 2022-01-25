package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Following {
    // public static final double K_TX = 1.;
    public static final double K_TA = 1.;
    
    public static final double TX_P = 0.0035;

    public static PID txPID;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 2);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        System.out.println(tv);
        if (tv == 1) {
            double tx = Robot.limelightSubsystem.getTableData(table, "tx");
            System.out.println("tx: " + Double.toString(tx));
            System.out.println("PID val: " + Double.toString(txPID.getOutput(tx, 0)));
            Robot.driveSubsystem.setSpeedForwardAngle(0.1, txPID.getOutput(tx, 0));
        } else {
            Robot.driveSubsystem.setSpeed(0,0);
        }
        // double ta = Robot.limelightSubsystem.getTableData(table, "ta");

        // double distance = ta * K_TA;
    }
}
