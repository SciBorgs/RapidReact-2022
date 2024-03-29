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

public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      FlywheelSubsystem flywheel,
      TurretSubsystem turret,
      String initialPos) {

    // init
    addCommands(
        new InstantCommand(intake::toggleArm, intake),
        new InstantCommand(intake::startSuck, intake));

    // shoot from tarmac
    addCommands(
        new Shoot(flywheel, hopper),
        new TurnDegrees(180, drive),
        new DriveUntilIntake(drive, intake));

    addCommands(
        new TurnDegrees(180, drive),
        new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage1", true),
        new Shoot(flywheel, hopper));

    if (initialPos == "2") addCommands(new TurnDegrees(180, drive));

    addCommands(
        new DriveRamsete(drive, "Pos" + initialPos + "_5Ball_Stage2", false),
        new TurnDegrees(180, drive),
        new Shoot(flywheel, hopper),
        new InstantCommand(intake::stopSuck, intake));
  }
}
