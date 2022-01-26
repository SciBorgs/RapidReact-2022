package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.Averager;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Deque;
import java.util.LinkedList;

public class Following {
    // public static final double K_TX = 1.;
    public static final double K_TA = 1.;
    
    public static final double TX_P = 0.035;
    public static final double TA_P = 0.4;

    public static PID txPID, taPID;

    public static Averager TA_Average = new Averager(5);

    static {
        txPID = new PID(TX_P, 0, 0);
        taPID = new PID(TA_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 2);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");
        double ta = Robot.limelightSubsystem.getTableData(table, "ta");
    
        double forward = -taPID.getOutput(Math.exp(-TA_Average.getAverageTA(ta)), 0);
        //System.out.println(tv);
        if (tv == 1) {
            Robot.driveSubsystem.setSpeedForwardAngle(forward, txPID.getOutput(tx, 0));
        } else {
            System.out.println("go nowhere");
        }
    }
}
