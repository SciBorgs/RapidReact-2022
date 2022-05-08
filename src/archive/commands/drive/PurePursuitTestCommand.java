// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.autoProfile.AutoProfile;
// import frc.robot.Robot;
// import frc.robot.controllers.PurePursuitController;

// public class PurePursuitTestCommand extends CommandBase {
//     private PurePursuitController pp;

//     @Override
//     public void initialize() {
//         this.pp = new PurePursuitController(AutoProfile.PATH_TEST, 0.201155, 0.0);
//     }

//     @Override
//     public void execute() {
//         pp.move();
//     }

//     @Override
//     public boolean isFinished() {
//         return this.pp.isFinished();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Robot.driveSubsystem.setSpeed(0.0, 0.0);
//     }
// }