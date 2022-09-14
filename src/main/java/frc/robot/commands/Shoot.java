package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Shoot extends SequentialCommandGroup {
  public Shoot(FlywheelSubsystem flywheel, HopperSubsystem hopper, VisionSubsystem vision) {
    vision.reset();
    addCommands(
        parallel(
            new RunCommand(
                () ->
                    flywheel.setTargetFlywheelSpeed(ShooterConstants.getRPM(vision.getDistance())),
                flywheel),
            new WaitUntilCommand(flywheel::atTargetRPM)),
        new WaitCommand(ShooterConstants.DOUBLE_BALL_TIMEOUT),
        new InstantCommand(flywheel::stopFlywheel, flywheel));
  }
}
