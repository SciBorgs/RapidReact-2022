package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveCommand extends InstantCommand {
    public void execute() {
        Robot.driveSubsystem.driveRobot(Robot.oi.leftStick, Robot.oi.rightStick, 1);
    }
}