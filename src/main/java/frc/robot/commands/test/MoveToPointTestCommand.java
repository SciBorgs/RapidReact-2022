package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controllers.MoveToPointController;
import frc.robot.util.DelayedPrinter;

public class MoveToPointTestCommand extends CommandBase{
    private MoveToPointController controller;
    private DelayedPrinter printer;

    public MoveToPointTestCommand() {
        super();
        this.printer = new DelayedPrinter(1000);
    }

    @Override
    public void initialize() {
        this.controller = new MoveToPointController(Constants.POINT_TEST);
    }

    @Override
    public void execute() {
        this.controller.move();
        this.printer.print("RUNNING COMMAND!!!!!!!!!!");
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Stopped");
        Robot.driveSubsystem.setSpeed(0, 0);
    }
}
