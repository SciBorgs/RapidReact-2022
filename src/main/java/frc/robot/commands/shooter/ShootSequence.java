package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.turret.ResetTurretCommand;

public class ShootSequence extends SequentialCommandGroup {
    public ShootSequence() {
        addCommands(
            // new AimCommandGroup(),
            new ShootCommandGroup()
            // new ResetTurretCommand()
        );
        // log shots so we can get default angle
        // System.out.println(Robot.shooterSubsystem.getHoodAngle());
    }
}
