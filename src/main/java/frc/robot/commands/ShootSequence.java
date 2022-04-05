package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.AimHoodCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.turret.ResetTurretCommand;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence() {
        addCommands(
            new AimHoodCommand()
            // new AimCommandGroup(),
            // new ShootCommandGroup(),
            // new ResetTurretCommand()
        );
    }
}
