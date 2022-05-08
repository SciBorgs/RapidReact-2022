// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.commands.hopper.StartElevatorCommand;

// import frc.robot.subsystems.LimeLightSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.HopperSubsystem;

// public class ShootCommandGroup extends ParallelDeadlineGroup  {
//     public ShootCommandGroup(ShooterSubsystem shooterSubsystem, LimeLightSubsystem limeLightSubsystem, HopperSubsystem hopperSubsystem) {
//         super(new ShootCommand(shooterSubsystem, limeLightSubsystem), new StartElevatorCommand(hopperSubsystem));
//     }
    
// }
