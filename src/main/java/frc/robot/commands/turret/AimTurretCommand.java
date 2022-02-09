package frc.robot.commands.turret;

import frc.robot.Robot;
import frc.robot.controllers.FollowTape;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimTurretCommand extends CommandBase {
    public AimTurretCommand() {
        super();
        this.addRequirements(Robot.limelightSubsystem);
    }

    @Override
    public void execute() {
        FollowTape.follow();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}