package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;

// Meant to be called at the beginning of Auto in front of ball. Has not been tested yet.
public class DriveUntilIntake extends CommandBase {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    public DriveUntilIntake(DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveRobot(DriveMode.TANK, 0.3, 0.3);
    }

    @Override
    public boolean isFinished() {
        return intake.hasBall();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobot(DriveMode.TANK, 0, 0);
    }

}
