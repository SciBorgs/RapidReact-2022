package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ResetTurretCommand extends CommandBase {
    private TurretSubsystem turret;

    public ResetTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setTargetAngle(0);
    }

    @Override
    public boolean isFinished() {
        return turret.isAtTarget();
    }
}
