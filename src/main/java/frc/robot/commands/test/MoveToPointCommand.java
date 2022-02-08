package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPoint;
import frc.robot.Constants;

public class MoveToPointAlphaCommand  extends CommandBase{
    private MoveToPoint controller;

    @Override
    public void initialize() {
        this.controller = new MoveToPoint(new Point(10, 0));
    }

    @Override
    public void execute() {
        if (controller.isFacingPoint()) { controller.move(); }
        else { controller.turn(); }
    }

    @Override
    public boolean isFinished() {
        return controller.isFacingPoint() && controller.hasArrived();
    }
}
