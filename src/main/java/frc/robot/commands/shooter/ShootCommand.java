package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CSVUtils.CSVWriter;

public class ShootCommand extends CommandBase {
    private static final double LIMIT = 100*Constants.WHEEL_CIRCUMFERENCE; // number of rotations before the command ends
    private static final double CLOSE = 0.3, MIDDLE = 0.5, FAR = 0.7;
    private static final double CLOSE_BOUND = 1, FAR_BOUND = 5;

    @Override
    public void initialize() {
        Robot.shooterSubsystem.resetDistanceSpun();
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.runFlywheel(getSpeed(Robot.shooterSubsystem.getDistance()));
        writeData();
    }

    public void writeData() {
        CSVWriter writer = new CSVWriter("/home/lvuser/logging.csv");
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
        // if (distance < CLOSE_BOUND)
        //     return CLOSE;
        // if (distance < FAR_BOUND)
        //     return MIDDLE;
        // return FAR;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("WE LEAVE");
    }

    @Override
    public boolean isFinished() {
        return false;
        // return Robot.shooterSubsystem.getDistanceSpun() > (double) LIMIT;
    }
}