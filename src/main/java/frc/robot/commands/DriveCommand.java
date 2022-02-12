package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class DriveCommand extends InstantCommand {
    // private DigitalInput limitSwitch = new DigitalInput(PortMap.TOP_LIMIT_SWITCH);

    public void execute(){
        Robot.driveSubsystem.moveRobot(Robot.oi.joystickLeft, Robot.oi.joystickRight, 1.0);
    }

}