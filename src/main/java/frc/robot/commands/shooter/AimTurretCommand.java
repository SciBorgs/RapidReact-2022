package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretCommand extends CommandBase {
    private TurretSubsystem turret;
    private VisionSubsystem limelight;

    public AimTurretCommand(TurretSubsystem turret, VisionSubsystem limelight) {
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret, limelight);
    }
    
    @Override
    public void execute() {
        limelight.setCameraParams(limelight.getTable(), "pipeline", 0);
        NetworkTable table = limelight.getTable();
        double tv = limelight.getTableData(table, "tv");
        double tx = limelight.getTableData(table, "tx");

        if (tv == 1) {
            turret.setTargetAngle(turret.getCurrentAngle() + tx);
        }
    }
    
    @Override
    public boolean isFinished() {
        return turret.isAtTarget();
    }
}
