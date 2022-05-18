package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunTelescopeCommand extends CommandBase {
    private ClimberSubsystem climberSubsystem;
    private boolean reversed;

    public RunTelescopeCommand(ClimberSubsystem climber, boolean reversed) {
        this.climberSubsystem = climber;
        this.reversed = reversed;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void execute() {
        climberSubsystem.runTelescope(this.reversed);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopTelescope(); 
    }
}
