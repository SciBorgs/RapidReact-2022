package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;
import frc.robot.util.PID;
import frc.robot.util.SciMath;

public class FollowTape {

    public static final double TX_P = 0.035;
    public static final double TX_WEIGHT = 0.1;

    private static PID txPID;
    private static double txAvr;
    private static int unknownCount = 0;
    
    public static final int UNKNOWN_LIMIT = 60;
    public static final double UNKNOWN_ANGLE = 5;

    private static double targetAngle = 0;

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
            targetAngle = getNewAngle(txAvr);
            unknownCount = 0;
        } else if (unknownCount > UNKNOWN_LIMIT) {
            targetAngle = getNewAngle(Robot.turretSubsystem.getDirection() * UNKNOWN_ANGLE);
        } else {
            unknownCount++;
        }
        
        double turn = txPID.getOutput(targetAngle, Robot.turretSubsystem.getAngle());
        System.out.println("Target Angle: " + targetAngle);
        System.out.println("Traveled: " + Robot.turretSubsystem.getAngle());
        System.out.println(turn + "\n---");
        if (turn > 0.5) {
            turn = 0.5;
            System.out.println("turn > 0.5");
        } else if (turn < -0.5) {
            turn = -0.5;
            System.out.println("turn < -0.5");
        }

        Robot.turretSubsystem.turn(turn);
    }
    
    // returns the angle that the robot is about to go to
    private static double getNewAngle(double newAngle) {
        return SciMath.normalizeInRange(
            SciMath.normalizeAngle(Robot.turretSubsystem.getAngle() + newAngle),
            -1,
            1
        );
    }
}
