package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveRamsete;
import frc.robot.commands.Turn180;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FenderTwoBallAuto extends SequentialCommandGroup {
    public FenderTwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, FlywheelSubsystem shooter, TurretSubsystem turret, String pos) {
        // Init
        addCommands(
            new InstantCommand(() -> shooter.setTargetFlywheelSpeed(ShooterConstants.FENDER_SPEED), shooter),
            new InstantCommand(intake::toggleArm, intake),
            new InstantCommand(intake::startSuck, intake)
        );

        // Grab second ball, go to fender, shoot
        addCommands(
            new DriveRamsete(drive, "Pos_" + pos + "_2Ball_Fender", true),
            new Turn180(drive),
            new InstantCommand(hopper::startElevator, hopper),
            new WaitCommand(ShooterConstants.DOUBLE_BALL_TIMEOUT),
            new InstantCommand(hopper::stopElevator, hopper),
            new InstantCommand(shooter::stopFlywheel, shooter)
        );

        // Drive off tarmac, end
        addCommands(
            new Turn180(drive),
            new DriveRamsete(drive, "DriveOffTarmac", false),
            new InstantCommand(intake::stopSuck),
            new InstantCommand(shooter::stopFlywheel, shooter)
        );
    }
}
