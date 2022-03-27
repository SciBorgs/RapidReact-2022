package frc.robot.commands.shooter;

import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;

public class AimHoodCommand extends CommandBase {

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Robot.shooterSubsystem.moveHood(Robot.shooterSubsystem.functionAngle());
        // double angle = Robot.shooterSubsystem.getCurrentHoodAngle();
        // System.out.println("Angle: " + angle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
