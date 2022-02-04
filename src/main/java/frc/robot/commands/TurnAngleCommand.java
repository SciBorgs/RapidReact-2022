package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.util.PID;
import frc.robot.util.Point;

public class TurnAngleCommand extends CommandBase {
    private static final double EPSILON = 1E-6;
    private PID anglePid;
    private double targetAngle;

    public TurnAngleCommand(Point p) {
        double dx = p.getX() - Robot.localizationSubsystem.getX();
        double dy = p.getY() - Robot.localizationSubsystem.getY();
        this.targetAngle = Math.atan2(dy, dx);
        this.anglePid = new PID(1, 0, 0);
    }

    public void execute() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        double output = this.anglePid.getOutput(targetAngle, currentAngle);
        Robot.driveSubsystem.setSpeedForwardAngle(0.1, output);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = Robot.localizationSubsystem.getAngle();
        return Math.abs(targetAngle - currentAngle) < EPSILON;
    }
}