package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

public class DriveCommand extends InstantCommand {
    public void execute() {
        Robot.driveSubsystem.driveRobot(DriveMode.TANK, Robot.oi.leftStick, Robot.oi.rightStick);
    }
}