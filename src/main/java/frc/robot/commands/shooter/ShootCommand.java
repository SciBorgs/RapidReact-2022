// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.LimeLightSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.util.CSVUtils.CSVWriter;

// public class ShootCommand extends CommandBase {
//     private ShooterSubsystem shooterSubsystem;
//     private LimeLightSubsystem limeLightSubsystem; //TODO move logging to different command
//     private final double LIMIT = 100 * Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends
//     private final double POWER = 0.5;

//     public ShootCommand(ShooterSubsystem shooterSubsystem, LimeLightSubsystem limeLightSubsystem) {
//         this.shooterSubsystem = shooterSubsystem;
//         this.limeLightSubsystem = limeLightSubsystem;
//         addRequirements(shooterSubsystem, limeLightSubsystem);
//     }

//     @Override
//     public void initialize() {
//         shooterSubsystem.resetDistanceSpun();
//         shooterSubsystem.setTargetFlywheelSpeed(POWER);
//     }

//     @Override
//     public void execute() {
//         // writeData();
//     }

//     public void writeData() {
//         CSVWriter writer = new CSVWriter("/home/lvuser/logging.csv");
//         writer.addData(
//             shooterSubsystem.translateFromEncoder(shooterSubsystem.getCurrentHoodAngle()),
//             limeLightSubsystem.getDistance(),
//             POWER,
//             "",
//             System.currentTimeMillis()
//         );
//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooterSubsystem.stopFlywheel();
//         System.out.println("WE LEAVE");
//     }

//     @Override
//     public boolean isFinished() {
//         return shooterSubsystem.getDistanceSpun() > (double) LIMIT;
//     }
// }