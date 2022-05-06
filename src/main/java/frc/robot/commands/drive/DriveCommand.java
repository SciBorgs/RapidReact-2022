package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier left;
    private final DoubleSupplier right;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier left, DoubleSupplier right) {
        this.driveSubsystem = driveSubsystem;
        this.left = left;
        this.right = right;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveRobot(DriveMode.TANK, left.getAsDouble(), right.getAsDouble());
    }
}
