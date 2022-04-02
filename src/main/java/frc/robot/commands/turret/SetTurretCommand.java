package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetTurretCommand extends CommandBase {
    private final double target;
    private static final double LENIENCY = 0.25;

    public SetTurretCommand(double target) {
        this.target = target;
    }
    
    @Override
    public void execute() {
        Robot.turretSubsystem.pointTowardsTarget(this.target);
    }
    
    @Override
    public boolean isFinished() {
        if (Math.abs(Robot.turretSubsystem.getAngle() - this.target) < LENIENCY) {
            Robot.turretSubsystem.stop();
            return true;
        }
        return false;
    }
}
