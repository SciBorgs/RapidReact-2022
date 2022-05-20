package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeArm extends InstantCommand {

    private IntakeSubsystem intake;

    public ToggleIntakeArm(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.toggleArm();
    }

}
