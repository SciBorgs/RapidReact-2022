package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveUntilIntake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
  public FourBallAuto(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      FlywheelSubsystem flywheel,
      TurretSubsystem turret,
      String initialPos) {

    addCommands(
        new InstantCommand(intake::toggleArm, intake),
        new InstantCommand(intake::startSuck, intake));

    addCommands(
        new DriveUntilIntake(drive, intake),
        new TurnDegrees(180, drive),
        new Shoot(flywheel, hopper));

    if (initialPos == "1") addCommands(new TurnDegrees(180, drive));

    addCommands(
        new DriveRamsete(drive, "Pos" + initialPos + "_4Ball", true),
        new TurnDegrees(180, drive),
        new Shoot(flywheel, hopper),
        new InstantCommand(intake::stopSuck, intake));
  }
}
