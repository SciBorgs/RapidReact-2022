package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveCommand extends InstantCommand {
    public void execute(){
        Robot.driveSubsystem.moveRobot(Robot.oi.joystickLeft, Robot.oi.joystickRight, 1.0);
    }
}