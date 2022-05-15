package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimHoodCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;

    public AimHoodCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double distance = vision.getDistance();
        double angle = ShooterConstants.getHoodAngle(distance);
        shooter.setTargetHoodAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return shooter.isAtTarget();
    }
}
