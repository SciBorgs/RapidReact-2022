package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.3;

    public static PID txPID;
    public static double taAvr;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");

        if (tv == 1) {
            Robot.turretSubsystem.turn(txPID.getOutput(tx * TX_WEIGHT, 0));
            //added the k constant to the tx... i think this is the same value as last time (not so sure)
        } else {
            Robot.turretSubsystem.turn(0.5);
        }

        /*
        if (tv == 1) {
            Robot.turretSubsystem.setTurretSpeed(txPID.getOutput(tx * TX_K, 0));
        } else {
            Robot.turretSubsystem.setTurretSpeed(0.5);
        }

        Again, this is to account for the fact that there is 
        */
    }
    

}
