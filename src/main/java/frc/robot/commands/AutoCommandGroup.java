package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ShootSequence;
import frc.robot.commands.AutoDriveCommand;

// Command group for 1-ball Auto (shoot preloaded ball then move off tarmac)
public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup() {
        addCommands(
            new ShootSequence(),
            new AutoDriveCommand(-0.21).withTimeout(2)
        );
    }
}