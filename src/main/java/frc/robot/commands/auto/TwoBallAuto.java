package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShootSequence.Target;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto (RobotContainer robot, String initialPos) {
        addCommands(
            new ParallelCommandGroup(
                new IntakeCommandGroup(robot.intakeSubsystem, robot.hopperSubsystem),
                new DriveRamsete(robot.driveSubsystem, "Pos" + initialPos + "_2Ball")
            ),
            new ShootSequence(robot.shooterSubsystem, robot.turretSubsystem, robot.hopperSubsystem, robot.limelightSubsystem, Target.HIGH)
        );
    }
}
