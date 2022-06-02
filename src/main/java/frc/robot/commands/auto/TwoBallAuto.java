package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision,
            ShooterSubsystem shooter, TurretSubsystem turret) {

        addCommands(
                new InstantCommand(
                        () -> intake.toggleArm(),
                        intake),
                new InstantCommand(
                        () -> intake.startSuck(),
                        intake));

        addCommands(
                new DriveUntilIntake(drive, intake),
                new Turn180(drive),
                new Shoot(
                        () -> ShooterConstants.getRPM(vision.getDistance()),
                        () -> ShooterConstants.getHoodAngle(vision.getDistance()),
                        () -> vision.getXOffset(),
                        shooter,
                        turret,
                        hopper),
                new InstantCommand(
                        () -> intake.stopSuck(),
                        intake));
    }
}
