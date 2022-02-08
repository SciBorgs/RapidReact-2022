package frc.robot.controllers;

import frc.robot.Robot;
import frc.robot.util.PID;

import java.util.Currency;

import edu.wpi.first.networktables.NetworkTable;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.5;

    public static PID txPID;

    private static int unknown_count = 0;
    private static final int UNKNOWN_LIMIT = 60;
    public static double txAvr;
    private static int direction = 1;
    private static double unknown_speed = 0.3;

    // until construction finishes
    // not to be imposing or anything
    private static int max_rotation = 500;
    private static int current_rotation = 0;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");
        
        if (tv == 1) {
            txAvr = TX_WEIGHT * tx + (1 - TX_WEIGHT) * txAvr;
            double turn = TX_WEIGHT * txPID.getOutput(txAvr, 0);
            Robot.turretSubsystem.turn(turn);
            if (turn > 0) {
                direction = 1;
                current_rotation++;
            }
            if (turn < 0) {
                direction = -1;
                current_rotation--;
            }
            unknown_count = 0;
        } else if (unknown_count < UNKNOWN_LIMIT) {
            Robot.turretSubsystem.turn(direction * unknown_speed);
        } else {
            unknown_count++;
        }
    }
    
}
