package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controllers.MoveToPoint;
import frc.robot.util.DelayedPrinter;

public class MoveToPointTestCommand extends CommandBase{
    private MoveToPoint controller;
    private final long start;
    private DelayedPrinter printer;

    public MoveToPointTestCommand() {
        super();
        start = System.currentTimeMillis();
        this.printer = new DelayedPrinter(1000);
    }

    @Override
    public void initialize() {
        this.controller = new MoveToPoint(Constants.POINT_TEST);
    }

    @Override
    public void execute() {
        this.controller.move();
        this.printer.print("RUNNING COMMAND!!!!!!!!!!");
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived() || System.currentTimeMillis() - start > 100000;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Stopped");
        Robot.driveSubsystem.setSpeed(0, 0);
    }
}
