package frc.robot.commands.turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;
    private LimeLightSubsystem limeLightSubsystem;
    private static int unknownCount = 0;
    public static final int UNKNOWN_LIMIT = 60;

    public AimTurretCommand(TurretSubsystem turretSubsystem, LimeLightSubsystem limeLightSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        addRequirements(turretSubsystem, limeLightSubsystem);
    }
    
    @Override
    public void execute() {
        limeLightSubsystem.setCameraParams(limeLightSubsystem.getTable(), "pipeline", 0);
        NetworkTable table = limeLightSubsystem.getTable();
        double tv = limeLightSubsystem.getTableData(table, "tv");
        double tx = limeLightSubsystem.getTableData(table, "tx");

        if (tv == 1) {
            unknownCount = 0;
            turretSubsystem.pointTowardsTarget(tx);
        } else if (unknownCount > UNKNOWN_LIMIT) {
            turretSubsystem.stop();
        } else {
            unknownCount++;
        }
    }
    
    @Override
    public boolean isFinished() {
        if (Math.abs(turretSubsystem.getAngle() - turretSubsystem.getTarget()) < 0.1) {
            turretSubsystem.stop();
            return true;
        }
        return false;
    }
}
