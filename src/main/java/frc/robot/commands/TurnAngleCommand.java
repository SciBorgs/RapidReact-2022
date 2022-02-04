package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;

public class TurnAngleCommand extends CommandBase {

    public void execute() {
        double dx = 5, dy = 5;
        double targetAngle = Math.atan2(dy, dx);

    }



    @Override
    public boolean isFinished() {
        return false;
    }
}