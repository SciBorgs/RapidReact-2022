package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class RumbleCommand extends InstantCommand {
    public void execute() {
        if(Robot.driveSubsystem.isStalling()) Robot.RumbleSubsystem.rumble();
    }
}