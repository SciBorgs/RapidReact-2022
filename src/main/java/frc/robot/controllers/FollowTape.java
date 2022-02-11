package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.util.PID;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.5;

    private static PID txPID;
    private static double txAvr;
    private static int unknownCount = 0;
    
    public static final int UNKNOWN_LIMIT = 60;
    public static final double UNKNOWN_SPEED = 0.3;

    static {
        txPID = new PID(TX_P, 0, 0);
    }

    public static void follow() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");
        double wrap = getReferenceAngle(totalAngle(tx)) - tx;
        
        if (tv == 1) {
            txAvr = TX_WEIGHT * wrap + (1 - TX_WEIGHT) * txAvr;
            double turn = TX_WEIGHT * txPID.getOutput(txAvr, 0);
            Robot.turretSubsystem.turn(turn);
            unknownCount = 0;
        } else if (unknownCount < UNKNOWN_LIMIT) {
            Robot.turretSubsystem.turn(Robot.turretSubsystem.getDirection() * UNKNOWN_SPEED);
        } else {
            unknownCount++;
        }
    }
    
    private static double totalAngle(double newAngle) {
        return Robot.turretSubsystem.getAngle() + newAngle;
    }

    private static double getReferenceAngle(double angle) {
        if (angle > 180)
            angle *= -1;
        return angle % 180;
    }
}
