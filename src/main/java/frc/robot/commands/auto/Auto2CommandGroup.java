// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// // import frc.robot.commands.auto.AutoDriveCommand;
// import frc.robot.commands.drive.SpinCommand;
// import frc.robot.commands.shooter.ShootCommand;
// import frc.robot.commands.shooter.ShootCommandGroup;
// import frc.robot.commands.shooter.ShootSequence;

// // Command group for 2-ball Auto
// public class Auto2CommandGroup extends SequentialCommandGroup {
//     public Auto2CommandGroup() {
//         addCommands(
//             new DriveUntilIntakeCommand(0.2),
//             new SpinCommand(Math.PI),
//             new ShootCommandGroup()
//             // IMPORTANT: REMEMBER TO ADJUST AUTODRIVECOMMAND VALUE BASED ON WHAT WAY YOU ARE FACING 
//         );
//     }
// }
