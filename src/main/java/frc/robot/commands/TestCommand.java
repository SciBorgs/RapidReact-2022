package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends SequentialCommandGroup {

    public TestCommand(TurretSubsystem turret, ShooterSubsystem shooter) {
        addCommands(
            parallel(
                new InstantCommand(
                    () -> turret.setTargetAngle(15.0),
                    turret
                ),
                new InstantCommand(
                    () -> shooter.setTargetHoodAngle(15.0),
                    shooter
                )
            )
        );
        addCommands(
            new InstantCommand(
                () -> shooter.setTargetFlywheelSpeed(60),
                shooter
            ).withTimeout(5),
            new InstantCommand(
                () -> shooter.setTargetFlywheelSpeed(0)
            )
        );

    }

}
