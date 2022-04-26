package frc.robot.commands.turret;

import frc.robot.Robot;
import frc.robot.commands.SubsystemCommand;

public class SetTurretCommand extends SubsystemCommand {
    private final double target;
    private static final double LENIENCY = 0.25;

    public SetTurretCommand(double target) {
        super(Robot.turretSubsystem);
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
