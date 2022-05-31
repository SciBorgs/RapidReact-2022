package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ToggleCompressorCommand extends CommandBase {
    private PneumaticsSubsystem pneumaticsSubsystem;

    public ToggleCompressorCommand(PneumaticsSubsystem pneumatics) {
        this.pneumaticsSubsystem = pneumatics;
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.start();
    }
    
    @Override
    public void end(boolean interruted) { 
        pneumaticsSubsystem.stop();
    }
}
