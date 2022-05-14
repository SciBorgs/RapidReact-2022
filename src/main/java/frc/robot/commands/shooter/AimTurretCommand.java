package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimTurretCommand extends CommandBase {
    private TurretSubsystem turret;
    private VisionSubsystem vision;

    public AimTurretCommand(TurretSubsystem turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }
    
    @Override
    public void execute() {
        if (vision.hasTarget()) {
            turret.setTargetAngle(turret.getCurrentAngle() + vision.getXOffset());
        }
    }
    
    @Override
    public boolean isFinished() {
        return turret.isAtTarget();
    }
}
