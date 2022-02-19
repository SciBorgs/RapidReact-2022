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
        Robot.shooterSubsystem.moveHood(5);
        double angle = Robot.shooterSubsystem.getHoodAngle();
        System.out.println("Angle: " + angle);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
