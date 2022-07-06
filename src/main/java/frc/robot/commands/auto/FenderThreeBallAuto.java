package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FenderThreeBallAuto extends SequentialCommandGroup {
    public FenderThreeBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, FlywheelSubsystem shooter, TurretSubsystem turret, String pos) {
        // start actions
        addCommands(
            new InstantCommand(intake::toggleArm, intake),
            new InstantCommand(intake::startSuck, intake),
            new InstantCommand(() -> shooter.setTargetFlywheelSpeed(ShooterConstants.FENDER_SPEED), shooter)
        );

        // Grab second ball, go to fender, shoot
        addCommands(
            new DriveRamsete(drive, "Pos_" + pos + "_2Ball_Fender", true),
            new Turn180(drive),
            new InstantCommand(hopper::startElevator, hopper),
            new WaitCommand(ShooterConstants.DOUBLE_BALL_TIMEOUT),
            new InstantCommand(hopper::stopElevator, hopper)
        );

        // Get third ball, shoot
        addCommands(
            new Turn180(drive),
            new DriveRamsete(drive, "Pos_" + pos + "_3Ball_Fender", false),
            new Turn180(drive),
            new InstantCommand(hopper::startElevator, hopper),
            new WaitCommand(ShooterConstants.DOUBLE_BALL_TIMEOUT),
            new InstantCommand(hopper::stopElevator, hopper)
        );

        // Drive off tarmac, end
        addCommands(
            new Turn180(drive),
            new DriveRamsete(drive, "DriveOffTarmac", false),
            new InstantCommand(intake::stopSuck, intake),
            new InstantCommand(shooter::stopFlywheel, shooter)
        );
    }
}
