package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ShootSequence;
import frc.robot.commands.auto.DriveUntilIntakeCommand;
import frc.robot.commands.auto.SpinCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.AutoDriveCommand;

// Command group for 2-ball Auto
public class Auto2CommandGroup extends SequentialCommandGroup {
    public Auto2CommandGroup() {
        addCommands(
            new DriveUntilIntakeCommand(0.7),
            new SpinCommand(Math.PI),
            new ShootCommandGroup()
            // IMPORTANT: REMEMBER TO ADJUST AUTODRIVECOMMAND VALUE BASED ON WHAT WAY YOU ARE FACING 
        );
    }
}
