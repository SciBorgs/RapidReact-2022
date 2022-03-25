package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.turret.ResetTurretCommand;

public class ShootCommandGroup extends SequentialCommandGroup {
    public ShootCommandGroup() {
        addCommands(
            new AimCommandGroup(),
            new ShootCommand(),
            new ResetTurretCommand()
        );
    }
}
