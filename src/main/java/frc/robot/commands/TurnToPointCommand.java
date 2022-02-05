package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.util.PID;
import frc.robot.util.Point;

public class TurnToPointCommand extends CommandBase {
    private static final double EPSILON = 1E-6;
    private PID anglePid;
    private double targetAngle;

    public TurnToPointCommand(Point p) {
        double dx = p.x - Robot.localizationSubsystem.getPos().x;
        double dy = p.y - Robot.localizationSubsystem.getPos().y;
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