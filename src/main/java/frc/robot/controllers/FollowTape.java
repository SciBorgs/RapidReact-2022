package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.Averager;
import frc.robot.util.PID;
import edu.wpi.first.networktables.NetworkTable;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.3;

    public static PID txPID;

    private static int unknown_count = 0;
    private static final int UNKNOWN_LIMIT = 60;
    public static double txAvr;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");
        
        
        double turn = 0.0;
        if (tv == 1) {
            txAvr = TX_WEIGHT * tx + (1 - TX_WEIGHT) * txAvr;
            turn = TX_WEIGHT * txPID.getOutput(Math.exp(-txAvr), 0);
            unknown_count = 0;
        } else if (unknown_count > UNKNOWN_LIMIT) {
            turn = -0.3;
        } else {
            unknown_count++;
        }

        Robot.turretSubsystem.turn(turn);

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
