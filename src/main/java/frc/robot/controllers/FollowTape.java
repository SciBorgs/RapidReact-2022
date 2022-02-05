package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.Averager;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.5;

    public static PID txPID;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");

        if (tv == 1) {
            Robot.turretSubsystem.turn(TX_WEIGHT * txPID.getOutput(tx, 0));
        } else {
            Robot.turretSubsystem.turn(0.5);
        }
    }
    
}
