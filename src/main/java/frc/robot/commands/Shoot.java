package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        var isShooting = new Debouncer(VisionConstants.TIMESCALE, DebounceType.kBoth);
        // var prepare = new FunctionalCommand(
        //     () -> {},
        //     // periodic
        //     () -> {
        //         shooter.setTargetFlywheelSpeed(speed.getAsDouble());
        //         turret.setTargetAngle(turret.getCurrentAngle() + horizontalOffset.getAsDouble());
        //     },
        //     (interupted) -> {},
        //     // end condition
        // () -> isShooting.calculate( shooter.atTargetRPM() && turret.atTarget()),
        //     shooter,
        //     turret);
        addCommands(
            new InstantCommand(() -> shooter.setTargetFlywheelSpeed(0.7)), // spin up
            new WaitCommand(2),
            new InstantCommand(hopper::startElevator, hopper), // run elevator, might need double ball timeout
            new WaitCommand(2),
            new InstantCommand(
                () -> {
                    shooter.setTargetFlywheelSpeed(0);
                    turret.setTargetAngle(0);
                    hopper.stopElevator();
                }));
    }
}