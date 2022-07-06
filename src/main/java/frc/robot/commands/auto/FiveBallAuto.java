package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.DriveUntilIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    public FiveBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision,
            FlywheelSubsystem shooter, TurretSubsystem turret, String initialPos) {

        // init
        addCommands(
                new InstantCommand(
                        () -> intake.toggleArm(),
                        intake),
                new InstantCommand(
                        () -> intake.startSuck(),
                        intake));

        addCommands(
                new Shoot(
                        () -> ShooterConstants.getRPM(vision.getDistance()),
                        () -> ShooterConstants.getHoodAngle(vision.getDistance()),
                        () -> vision.getXOffset(),
                        shooter,
                        turret,
                        hopper),
                new Turn180(drive),
                new DriveUntilIntake(drive, intake));

        addCommands(
            new Turn180(drive),
                new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage1", true),
                new Shoot(
                        () -> ShooterConstants.getRPM(vision.getDistance()),
                        () -> ShooterConstants.getHoodAngle(vision.getDistance()),
                        () -> vision.getXOffset(),
                        shooter,
                        turret,
                        hopper));

        if (initialPos == "2")
            addCommands(new Turn180(drive));

        addCommands(
                new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage2", false),
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
