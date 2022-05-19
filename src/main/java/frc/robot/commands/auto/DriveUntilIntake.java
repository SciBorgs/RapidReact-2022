package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

// Meant to be called at the beginning of Auto in front of ball. Has not been tested yet.
public class DriveUntilIntake extends CommandBase {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private int currentBallCount;

    public DriveUntilIntake(DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;
        addRequirements(drive, intake);
    }

    @Override
    public void initialize() {
        currentBallCount = intake.getBallCount();
    }

    @Override
    public void execute() {
        drive.driveRobot(DriveMode.TANK, -0.2, -0.2); // drive backwards
    }

    @Override
    public boolean isFinished() {
        return intake.getBallCount() == currentBallCount+1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobot(DriveMode.TANK, 0, 0);
    }

}
