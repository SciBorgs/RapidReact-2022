package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ResetTurretCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;

    public ResetTurretCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.pointTowardsTarget(0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(turretSubsystem.getAngle()) < 0.1) {
            turretSubsystem.stop();
            return true;
        }
        return false;
    }
}
