package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualClimberCommand extends CommandBase {
    @Override
    public void execute() {
        Robot.climberSubsystem.setSpeed(Robot.oi.rightStick.getY());
    }
}
