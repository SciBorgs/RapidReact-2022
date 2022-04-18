package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.hopper.StartElevatorCommand;

public class ShootCommandGroup extends ParallelDeadlineGroup  {

    public ShootCommandGroup() {
        super(new ShootCommand(), new StartElevatorCommand());
    }
    
}
