package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.PathPlanner;

public class DriveRamsete extends RamseteCommand {
    private DriveSubsystem driveSubsystem;
    private Trajectory trajectory;

    public DriveRamsete(DriveSubsystem ds, Trajectory trajectory) {
        super(
            trajectory,
            ds::getPose,
            new RamseteController(),
            Constants.DriveConstants.feedForward,
            Constants.DriveConstants.kinematics,
            ds::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD),
            new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD),
            ds::tankDriveVolts,
            ds
        );
        
        this.driveSubsystem = ds;
        this.trajectory = trajectory;
    }

    public DriveRamsete(DriveSubsystem ds, String pathName) {
        this(ds, PathPlanner.loadPath(pathName, Constants.DriveConstants.maxVel, Constants.DriveConstants.maxAccel));
    }

    @Override
    public void initialize() {
        super.initialize();
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

}
