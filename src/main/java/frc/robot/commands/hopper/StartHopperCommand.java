package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class StartHopperCommand extends CommandBase {
    private HopperSubsystem hopperSubsystem;
    // WARNING: THIS COMMAND NEVER FINISHES BY ITSELF. USE WITH TIMEOUT OR OTHER END CONDITION.
    
    public StartHopperCommand(HopperSubsystem hopper) {
        this.hopperSubsystem = hopper;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void execute() {
        hopperSubsystem.startElevator();
        hopperSubsystem.startSuck();
    }

    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.stopElevator();
        hopperSubsystem.stopSuck();
    }
}
