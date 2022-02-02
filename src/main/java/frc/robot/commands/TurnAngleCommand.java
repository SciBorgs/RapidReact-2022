package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;

public class TurnAngleCommand extends InstantCommand {
    public void execute() {
        Robot.driveSubsystem.setSpeed(Robot.oi.joystickLeft.getY(),
                Robot.oi.joystickRight.getY());
    }

    @Override
    public boolean isFinished() {
        return 
    }
}