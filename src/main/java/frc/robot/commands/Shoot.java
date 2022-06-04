package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Shoot extends SequentialCommandGroup {
    /**
     * Initializes the shoot command group
     * @param speed flywheel rpm
     * @param angle hood angle
     * @param horizontalOffset distance for the turret to turn
     * @param shooter
     * @param turret
     * @param hopper
     */
    public Shoot(DoubleSupplier speed, DoubleSupplier angle, DoubleSupplier horizontalOffset, ShooterSubsystem shooter, TurretSubsystem turret, HopperSubsystem hopper) {
        var isShooting = new Debouncer(VisionConstants.TIMESCALE, DebounceType.kBoth);
        var prepare = new FunctionalCommand(
            () -> {},
            // periodic
            () -> {
                shooter.setTargetFlywheelSpeed(speed.getAsDouble());
                shooter.setTargetHoodAngle(angle.getAsDouble());
                turret.setTargetAngle(turret.getCurrentAngle() + horizontalOffset.getAsDouble());
            },
            (interupted) -> {},
            // end condition
            () -> isShooting.calculate(shooter.atTargetAngle() && shooter.atTargetRPM() && turret.atTarget()),
            shooter,
            turret);

        addCommands(
            prepare, // spin up
            new StartEndCommand(hopper::startElevator, hopper::stopElevator, hopper).withTimeout(ShooterConstants.SINGLE_BALL_TIMEOUT), // run elevator, might need double ball timeout
            new InstantCommand(
                () -> {
                    shooter.setTargetFlywheelSpeed(0);
                    shooter.setTargetHoodAngle(0);
                    turret.setTargetAngle(0);
                    hopper.stopElevator();
                }));
    }
}