package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManualTurretCommand extends CommandBase {
    @Override
    public void execute() {
        double val = Robot.oi.joystickLeft.getY();
        Robot.turretSubsystem.motor.set(val);
    }
}
