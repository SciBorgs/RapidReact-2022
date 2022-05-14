package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootSequence extends SequentialCommandGroup {
    private int timeout = 5;
    public enum Target {
        LOW,
        HIGH
    }

    public ShootSequence(DoubleSupplier rpm, DoubleSupplier angle, ShooterSubsystem shooter, TurretSubsystem turret, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem limelight, Target target) {
        switch(target) {
            case LOW:
                break;
            case HIGH:
                addCommands(
                    parallel(
                        new AimTurretCommand(turret, limelight),
                        new AimHoodCommand(shooter)
                    )
                );
        }
        
        double speed = 0.8; // TODO add variable speed
        addCommands(
            race(
                new StartEndCommand(
                    () -> shooter.setTargetFlywheelSpeed(speed),
                    () -> shooter.setTargetFlywheelSpeed(0),
                    shooter
                ).withTimeout(timeout),
                new StartEndCommand(
                    () -> hopper.startElevator(), 
                    () -> {hopper.stopElevator(); intake.decrementBallCount();}, //safeset place to decrement balls since when the elevator stops the ball should entirely not be in the hopper anymore
                    hopper
                )
            )
        );

        addCommands(
            new ResetTurretCommand(turret)
        );
    }
}
