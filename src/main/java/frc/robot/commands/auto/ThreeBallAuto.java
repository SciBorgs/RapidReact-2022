package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.intake.IntakeForever;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.intake.ToggleIntakeArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeBallAuto extends SequentialCommandGroup {
    public ThreeBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {
        
        addCommands(
            new ToggleIntakeArm(intake),
            new IntakeForever(intake)
        );

        addCommands(
            new ShootSequence(shooter, turret, hopper, limelight),
            new TurnToAngle(180, drive),
            new DriveUntilIntake(drive, intake)
        );

        if(initialPos == "2") addCommands(new TurnToAngle(0, drive));
        
        addCommands(
            new DriveRamsete(drive, "Pos" + initialPos + "_3Ball", true),
            new TurnToAngle(0, drive), // TODO: need to put accurate angle here
            new ShootSequence(shooter, turret, hopper, limelight),
            new IntakeStop(intake)
        );
    }
}
