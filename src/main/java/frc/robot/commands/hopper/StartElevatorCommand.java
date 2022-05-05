package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class StartElevatorCommand extends CommandBase {
    private HopperSubsystem hopperSubsystem;

    public StartElevatorCommand(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        this.addRequirements(hopperSubsystem);
    }

    @Override
    public void execute() {
        hopperSubsystem.startElevator();
    }

    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.stopElevator();
    }
}
