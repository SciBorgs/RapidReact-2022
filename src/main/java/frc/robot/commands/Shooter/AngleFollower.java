package frc.robot.commands.Shooter;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;

public class AngleFollower extends CommandBase {
    @Override
    public void execute(){
        Robot.shooterSubsystem.moveHood(21.5);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
