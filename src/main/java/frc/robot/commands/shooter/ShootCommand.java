package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;

    public ShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = vision.getDistance();
        double angle = ShooterConstants.getRPM(distance);
        shooter.setTargetFlywheelSpeed(angle);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetFlywheelSpeed(0);
    }
}
