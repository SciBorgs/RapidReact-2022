package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CSVUtils.CSVWriter;

public class ShootCommand extends CommandBase {
    private static final double LIMIT = 100*Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends
    private static final double CLOSE = 0.3, MIDDLE = 0.5, FAR = 0.7;

    @Override
    public void execute() {
        Robot.shooterSubsystem.runFlywheel(getSpeed(Robot.shooterSubsystem.getDistance()));
        writeData();
    }

    public void writeData() {
        CSVWriter writer = new CSVWriter("src/main/java/frc/robot/controllers/placeHolder.csv");
        writer.addData(
            Robot.shooterSubsystem.translateFromEncoder(Robot.shooterSubsystem.getHoodAngle()),
            Robot.shooterSubsystem.getDistance(),
            getSpeed(Robot.shooterSubsystem.getDistance()),
            "",
            System.currentTimeMillis()
        );
    }

    public double getSpeed(double distance) {
        return MIDDLE;
        // if (distance > )
    }


    @Override
    public boolean isFinished() {
        return Robot.shooterSubsystem.getDistanceSpun() > (double) LIMIT;
    }
}