package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.hopper.StartElevatorCommand;
import frc.robot.commands.shooter.ShootCommand;

public class ShootCommandGroup extends ParallelCommandGroup  {

    public ShootCommandGroup() {
        addCommands(
            new ShootCommand(),
            new StartElevatorCommand()
        );
    }
    
}
