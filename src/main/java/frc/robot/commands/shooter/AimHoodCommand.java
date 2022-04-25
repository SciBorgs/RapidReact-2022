package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.ShooterData;

public class AimHoodCommand extends CommandBase {
    public static final double ANGLE = 15;
    private double[][] hoodData;

    @Override
    public void initialize() {
        hoodData = ShooterData.getHoodAngleData("src/main/java/frc/robot/controllers/placeHolder.csv");
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.moveHood(ANGLE);

        // for averaging data, if we have time
        
        // double distance = Robot.limelightSubsystem.getDistance();
        // double angle = ShooterData.calcHoodAngle(hoodData, distance);
        // Robot.shooterSubsystem.moveHood(angle);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooterSubsystem.setHoodSpeed(0);
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(Robot.shooterSubsystem.getHoodAngle() - ANGLE) < 0.1) {
            Robot.shooterSubsystem.setHoodSpeed(0);
            return true;
        }
        return false;
    }
}
