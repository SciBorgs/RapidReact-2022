package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.FlywheelController;
import frc.robot.controllers.HoodAngleController;

public class ShootCommand extends CommandBase {
    private FlywheelController flywheelController;
    private HoodAngleController hoodAngleController;

    public ShootCommand() {
        flywheelController = new FlywheelController();
        hoodAngleController = new HoodAngleController();
    }
    @Override
    public void execute() {
        
    }
}
