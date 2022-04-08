package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveCommand extends InstantCommand {
    public void execute() {
        if (Robot.oi.isXbox) {
            Robot.driveSubsystem.driveRobot(Robot.oi.xboxController, 0.7);
        } else { 
            Robot.driveSubsystem.driveRobot(Robot.oi.leftStick, Robot.oi.rightStick, 0.7);
        }
    }
}