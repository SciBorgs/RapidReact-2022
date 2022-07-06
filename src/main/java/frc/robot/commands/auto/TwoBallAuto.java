package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveUntilIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, FlywheelSubsystem shooter, TurretSubsystem turret) {
        // init
        addCommands(
            new InstantCommand(intake::toggleArm, intake),
            new InstantCommand(intake::startSuck, intake)
        );
        
        // intake second ball and shoot
        addCommands(
            new DriveUntilIntake(drive, intake),
            new Turn180(drive),
            new InstantCommand(hopper::startElevator, hopper),
            new WaitCommand(ShooterConstants.DOUBLE_BALL_TIMEOUT),
            new InstantCommand(hopper::stopElevator, hopper)
        );

        // end
        addCommands(
            new InstantCommand(intake::stopSuck, intake),
            new InstantCommand(shooter::stopFlywheel, shooter)
        );
    }
}
