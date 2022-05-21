package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.util.BallCounter;

// Meant to be called at the beginning of Auto in front of ball. Has not been tested yet.
public class DriveUntilIntake extends CommandBase {
    private DriveSubsystem drive;
    private BallCounter count;
    private int currentBallCount;

    public DriveUntilIntake(DriveSubsystem drive, BallCounter count) {
        this.drive = drive;
        this.count = count;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currentBallCount = count.get();
    }

    @Override
    public void execute() {
        drive.driveRobot(DriveMode.TANK, 0.3, 0.3);
    }

    @Override
    public boolean isFinished() {
        return count.get() == currentBallCount+1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobot(DriveMode.TANK, 0, 0);
    }

}
