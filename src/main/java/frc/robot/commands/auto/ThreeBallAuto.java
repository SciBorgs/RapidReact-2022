package frc.robot.commands.auto;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ThreeBallAuto extends SequentialCommandGroup {
  public ThreeBallAuto(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      FlywheelSubsystem flywheel,
      TurretSubsystem turret,
      String initialPos) {

    addCommands(
        // Drive to first ball and drop intake
        // new DriveRamsete(drive, "Pos" + initialPos + "_3Ball_Stage1", true),
        // new InstantCommand(intake::startSuck, intake),
        // new InstantCommand(intake::toggleArm, intake),

        // // Turn around and shoot 2 balls
        // new TurnDegrees(180, drive),
        // new Shoot(flywheel, hopper),
        // new InstantCommand(intake::toggleArm, intake),

        // // Get 3rd ball, drop intake, shoot
        // new DriveRamsete(drive, "Pos" + initialPos + "3Ball_Stage2", false),
        // new TurnDegrees(180, drive),
        // new Shoot(flywheel, hopper),
        // new InstantCommand(intake::stopSuck, intake));
        new InstantCommand());
  }
}
