package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootSequence extends SequentialCommandGroup {
    private int timeout = 5;
    public enum Target {
        LOW,
        HIGH
    }

    public ShootSequence(ShooterSubsystem shooter, TurretSubsystem turret, IntakeSubsystem intake, HopperSubsystem hopper, VisionSubsystem vision, Target target) {
        vision.reset();
        switch(target) {
            case LOW:
                break;
            case HIGH:
                addCommands(
                    parallel(
                        new AimTurretCommand(turret, vision),
                        new AimHoodCommand(shooter, vision)
                    )
                );
        }
        
        addCommands(
            race(
                new ShootCommand(shooter, vision).withTimeout(timeout),
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
