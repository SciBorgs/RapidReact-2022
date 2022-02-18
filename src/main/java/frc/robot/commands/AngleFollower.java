package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;

public class AngleFollower {
    ShooterSubsystem shoot = new ShooterSubsystem();
    public void execute(){
        shoot.moveHood(21.5);
    }
}
