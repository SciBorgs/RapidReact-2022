// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.util.ShooterData;

// public class AimHoodCommand extends CommandBase {
//     private ShooterSubsystem shooterSubsystem;
//     public static final double ANGLE = 15;
//     private double[][] hoodData;

//     public AimHoodCommand(ShooterSubsystem shooterSubsystem) {
//         this.shooterSubsystem = shooterSubsystem;
//         hoodData = ShooterData.getHoodAngleData("src/main/java/frc/robot/controllers/placeHolder.csv");
//         addRequirements(shooterSubsystem);
//     }

//     @Override
//     public void execute() {
//         shooterSubsystem.moveHood(ANGLE);

//         // for averaging data, if we have time
        
//         // double distance = Robot.limelightSubsystem.getDistance();
//         // double angle = ShooterData.calcHoodAngle(hoodData, distance);
//         // Robot.shooterSubsystem.moveHood(angle);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooterSubsystem.setHoodSpeed(0);
//     }

//     @Override
//     public boolean isFinished(){
//         if (Math.abs(shooterSubsystem.getHoodAngle() - ANGLE) < 0.1) {
//             shooterSubsystem.setHoodSpeed(0);
//             return true;
//         }
//         return false;
//     }
// }
