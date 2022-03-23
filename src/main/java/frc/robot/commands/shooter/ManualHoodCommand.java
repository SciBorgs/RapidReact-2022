package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualHoodCommand extends CommandBase {
    @Override
    public void execute() {
        double val = Robot.oi.joystickLeft.getY();
        // System.out.println(val);
        // Robot.shooterSubsystem.moveVert(val);
    }
}
