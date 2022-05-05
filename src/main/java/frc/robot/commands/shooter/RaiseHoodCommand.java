package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RaiseHoodCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    public static final double SPEED = -0.07;

    public RaiseHoodCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setHoodSpeed(SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodSpeed(0); 
    }
}
