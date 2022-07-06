package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.DriveUntilIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeBallAuto extends SequentialCommandGroup {
    public ThreeBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, FlywheelSubsystem flywheel, TurretSubsystem turret, String initialPos) {
        
        addCommands(
            new InstantCommand(intake::toggleArm, intake),
            new InstantCommand(intake::startSuck, intake)
        );

        addCommands(
            new Shoot(flywheel, hopper, vision),
            new Turn180(drive),
            new DriveUntilIntake(drive, intake)
        );

        if(initialPos == "2") addCommands(new Turn180(drive));
        
        addCommands(
            new DriveRamsete(drive, "Pos" + initialPos + "_3Ball", true),
            new Turn180(drive),
            new Shoot(flywheel, hopper, vision),
            new InstantCommand(intake::stopSuck, intake)
        );
    }
}
