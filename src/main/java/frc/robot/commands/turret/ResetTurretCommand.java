package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ResetTurretCommand extends CommandBase {

    @Override
    public void execute() {
        Robot.turretSubsystem.pointTowardsTarget(0);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Robot.turretSubsystem.getAngle()) < 0.1) {
            Robot.turretSubsystem.stop();
            return true;
        }
        return false;
    }
}