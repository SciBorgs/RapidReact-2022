package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CSVUtils.CSVWriter;

public class ShootCommand extends CommandBase {
    private static final double LIMIT = 100*Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends
    private static final double POWER = 0.5;

    @Override
    public void initialize() {
        Robot.shooterSubsystem.resetDistanceSpun();
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.runFlywheel(POWER);
        writeData();
    }

    public void writeData() {
        CSVWriter writer = new CSVWriter("/home/lvuser/logging.csv");
        writer.addData(
            Robot.shooterSubsystem.translateFromEncoder(Robot.shooterSubsystem.getHoodAngle()),
            Robot.limelightSubsystem.getDistance(),
            POWER,
            "",
            System.currentTimeMillis()
        );
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooterSubsystem.stopFlywheel();
        System.out.println("WE LEAVE");
    }

    @Override
    public boolean isFinished() {
        return Robot.shooterSubsystem.getDistanceSpun() > (double) LIMIT;
    }
}