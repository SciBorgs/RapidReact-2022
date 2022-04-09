package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CSVUtils.CSVWriter;

public class ShootCommand extends CommandBase {
    private static double speed = 0.5; // speed of rotation
    private static final double LIMIT = 100*Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends

    @Override
    public void execute() {
        Robot.shooterSubsystem.runFlywheel(speed);
        CSVWriter writer = new CSVWriter("src/main/java/frc/robot/controllers/placeHolder.csv");
        writer.addData(
            Robot.shooterSubsystem.getHoodAngle(),
            Robot.shooterSubsystem.getDistance(),
            speed,
            null,
            System.currentTimeMillis()
        );
    }

    public void changeSpeed(double newSpeed) {
        speed = newSpeed;
    }

    @Override
    public boolean isFinished() {
        return Robot.shooterSubsystem.getDistanceSpun() > (double) LIMIT;
    }
}