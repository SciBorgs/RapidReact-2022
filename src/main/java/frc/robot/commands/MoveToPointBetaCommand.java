package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.MoveToPoint;
import frc.robot.Constants;

public class MoveToPointBetaCommand  extends CommandBase{
    private MoveToPoint controller;

    @Override
    public void initialize() {
        this.controller = new MoveToPoint(Constants.POINT_BETA);
    }

    @Override
    public void execute() {
        this.controller.move();
    }

    @Override
    public boolean isFinished() {
        return controller.hasArrived();
    }
}
