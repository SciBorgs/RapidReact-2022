package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.turret.AimTurretCommand;

import static frc.robot.util.Util.withAllWrapped;

public class AimCommandGroup extends ParallelCommandGroup {
    public AimCommandGroup() {
        addCommands(withAllWrapped(
            new AimTurretCommand()
            // hood aiming is manual for now
            // new AimHoodCommand()
        ));
    }
}
