// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.controllers.AlongAxisController;
// import frc.robot.Constants;
// import frc.robot.Robot;

// public class MoveUpToHubCommand extends CommandBase {
//     private AlongAxisController axisController;

//     private static final double DISTANCE_TOLERANCE = 0.05;

//     public MoveUpToHubCommand(double distance) {
//         this.axisController = new AlongAxisController(Constants.POINT_HUB, DISTANCE_TOLERANCE);
//         this.axisController.setTarget(distance);
//     }

//     @Override
//     public void execute() {
//         this.axisController.move();
//     }

//     @Override
//     public boolean isFinished() {
//         return this.axisController.isFinished();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Robot.driveSubsystem.setSpeed(0.0, 0.0);
//     }
// }
