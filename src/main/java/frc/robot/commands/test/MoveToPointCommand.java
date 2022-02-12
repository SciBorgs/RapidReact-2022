package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controllers.MoveToPoint;
import frc.robot.util.Point;

public class MoveToPointCommand extends CommandBase{
    private MoveToPoint controller;
    private final long start;

    public MoveToPointCommand() {
        super();
        start = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        this.controller = new MoveToPoint(new Point(0, 0));
    }

    @Override
    public void execute() {
        System.out.println("RUNNING COMMAND!!!!!!!!!!!!!!");
        this.controller.move();
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived() || System.currentTimeMillis() - start > 100000;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.driveSubsystem.setSpeed(0, 0);
    }
}
