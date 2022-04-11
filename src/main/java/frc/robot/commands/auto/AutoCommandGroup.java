package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoDriveCommand;
import frc.robot.commands.shooter.ShootSequence;

// Command group for 1-ball Auto (shoot preloaded ball then move off tarmac)
public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup() {
        addCommands(
            new ShootSequence(), // ** IF SHOOTER IS NOT USED DURING AUTO COMMENT OUT THIS LINE JUST IN CASE
            new AutoDriveCommand(0.21).withTimeout(2) 
            // IMPORTANT: REMEMBER TO ADJUST AUTODRIVECOMMAND VALUE BASED ON WHAT WAY YOU ARE FACING 
        );
    }
}