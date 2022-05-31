package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HighShot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeBallAuto extends SequentialCommandGroup {
    public ThreeBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, ShooterSubsystem shooter, TurretSubsystem turret, String initialPos) {
        
        addCommands(
                new InstantCommand(
                    () -> intake.toggleArm(),
                    intake
                ),
                new InstantCommand(
                    () -> intake.startSuck(),
                    intake
                ));

        addCommands(
            new HighShot(shooter, turret, hopper, limelight).withTimeout(ShooterConstants.SINGLE_BALL_TIMEOUT), // TODO we can't shoot on tarmac
            new TurnToAngle(180, drive),
            new DriveUntilIntake(drive, intake)
        );

        if(initialPos == "2") addCommands(new TurnToAngle(0, drive));
        
        addCommands(
            new DriveRamsete(drive, "Pos" + initialPos + "_3Ball", true),
            new TurnToAngle(0, drive), // TODO: need to put accurate angle here
            new HighShot(shooter, turret, hopper, limelight),
            new InstantCommand(
                    () -> intake.stopSuck(),
                    intake
                )
        );
    }
}
