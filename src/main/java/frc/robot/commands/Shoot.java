package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Shoot extends SequentialCommandGroup {
    /**
     * Initializes the shoot command group
     * @param speed flywheel rpm
     * @param horizontalOffset distance for the turret to turn
     * @param shooter
     * @param turret
     * @param hopper
     */
    public Shoot(DoubleSupplier speed, DoubleSupplier horizontalOffset, ShooterSubsystem shooter, TurretSubsystem turret, HopperSubsystem hopper) {
        addCommands(
            new InstantCommand(() -> shooter.setTargetFlywheelSpeed(speed.getAsDouble())), // spin up
            new InstantCommand(() -> turret.setTargetAngle(turret.getTargetAngle() + horizontalOffset.getAsDouble())),
            new WaitUntilCommand(turret::atTarget),
            new WaitCommand(ShooterConstants.FLYWHEEL_RAMP_TIMEOUT),
            new InstantCommand(hopper::startElevator, hopper), // run elevator, might need double ball timeout
            new WaitCommand(ShooterConstants.SINGLE_BALL_TIMEOUT),
            new InstantCommand(
                () -> {
                    shooter.setTargetFlywheelSpeed(0);
                    hopper.stopElevator();
                }));
    }
}