package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Util;

public class AimHoodCommand extends CommandBase {
    public static final double ANGLE = 15;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Robot.shooterSubsystem.moveHood(ANGLE);

        // for averaging data, if we have time
        // double[][] hoodData = Util.getHoodAngleData("src/main/java/frc/robot/controllers/placeHolder.csv");
        // double distance = Robot.shooterSubsystem.getDistance();
        // double angle = Util.calcHoodAngle(hoodData, distance);
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
