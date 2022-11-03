package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.DriveUntilIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
  public TwoBallAuto(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      FlywheelSubsystem flywheel,
      TurretSubsystem turret,
      String initialPos) {
    addCommands(
        // Drive to ball
        new DriveRamsete(drive, "Pos" + initialPos + "_2Ball", true),
        // Drop intake on ball
        new InstantCommand(intake::startSuck, intake),
        new InstantCommand(intake::toggleArm, intake),
        // Turn around and shoot
        new TurnDegrees(180, drive),
        new Shoot(flywheel, hopper),
        new InstantCommand(intake::stopSuck, intake));
  }
}
