package frc.robot.commands.turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AimTurretCommand extends CommandBase {
    private static int unknownCount = 0;
    public static final int UNKNOWN_LIMIT = 60;
    
    @Override
    public void execute() {
        Robot.limelightSubsystem.setCameraParams(Robot.limelightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = Robot.limelightSubsystem.getTable();
        double tv = Robot.limelightSubsystem.getTableData(table, "tv");
        double tx = Robot.limelightSubsystem.getTableData(table, "tx");

        if (tv == 1) {
            unknownCount = 0;
            Robot.turretSubsystem.pointTowardsTarget(tx);
        } else if (unknownCount > UNKNOWN_LIMIT) {
            Robot.turretSubsystem.stop();
        } else {
            unknownCount++;
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
