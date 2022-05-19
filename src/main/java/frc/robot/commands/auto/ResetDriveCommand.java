package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetDriveCommand extends InstantCommand {
    private DriveSubsystem drive;

    public ResetDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void execute() {
        drive.reset();
    }
}
