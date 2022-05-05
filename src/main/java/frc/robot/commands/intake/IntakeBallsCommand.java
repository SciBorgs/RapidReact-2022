package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallsCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private HopperSubsystem hopperSubsystem;

    public IntakeBallsCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(intakeSubsystem, hopperSubsystem);
    }
 
    @Override
    public void execute() {
        intakeSubsystem.startSuck();
        hopperSubsystem.startSuck();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopSuck();
        hopperSubsystem.stopSuck();
    }    
}
