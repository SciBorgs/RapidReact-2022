package frc.robot.commands.Shooter;

import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.controllers.HoodAngleController;

public class AimHoodCommand extends CommandBase {
    private HoodAngleController hoodAngleController;

    @Override
    public void initialize() {
        hoodAngleController = new HoodAngleController();
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.moveHood(Robot.shooterSubsystem.functionAngle());
        double angle = Robot.shooterSubsystem.getCurrentHoodAngle();
        System.out.println("Angle: " + angle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
