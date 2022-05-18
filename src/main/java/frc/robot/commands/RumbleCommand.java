package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RumbleSubsystem;

public class RumbleCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private RumbleSubsystem rumbleSubsystem;

    public RumbleCommand(DriveSubsystem drive, RumbleSubsystem rumble) {
        this.driveSubsystem = drive;
        this.rumbleSubsystem = rumble;
        // addRequirements(driveSubsystem, rumbleSubsystem);
    }
    public void execute() {
        if(driveSubsystem.isStalling()) rumbleSubsystem.rumble();
        else rumbleSubsystem.stopRumble();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}