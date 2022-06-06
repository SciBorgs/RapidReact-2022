package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.AimHoodCommand;
import frc.robot.commands.turret.AimTurretCommand;

public class AimCommandGroup extends ParallelCommandGroup {
    public AimCommandGroup() {
        addCommands(
            new AimTurretCommand()
            // hood aiming is manual for now
            // new AimHoodCommand()
        );
    }
}
