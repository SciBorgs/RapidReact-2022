// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.controllers.AlongAxisController;
// import frc.robot.autoProfile.AutoProfile;
// import frc.robot.Robot;

// public class AlongAxisTestCommand extends CommandBase {
//     private AlongAxisController axisController;
//     private double farDistance = 3;
//     private double closeDistance = -3; 
//     private boolean goingToFar; // these variable names are bad :(

//     private static final double DISTANCE_TOLERANCE = 1;

//     @Override
//     public void initialize() {
//         this.axisController = new AlongAxisController(AutoProfile.STARTING_POINT, 0, DISTANCE_TOLERANCE);
//         this.axisController.setTarget(farDistance);
//         this.goingToFar = false;
//     }

//     @Override
//     public void execute() {
//         if (this.axisController.atTarget()) {
//             goingToFar = !goingToFar;
//             setTarget();
//         }
//         this.axisController.move();
//     }

//     public void setTarget() {
//         if (goingToFar) this.axisController.setTarget(farDistance);
//         else this.axisController.setTarget(closeDistance);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Robot.driveSubsystem.setSpeed(0.0, 0.0);
//     }
// }
