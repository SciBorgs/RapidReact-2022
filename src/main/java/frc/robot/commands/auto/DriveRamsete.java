package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
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
            ds.getFeedforward(),
            ds.getKinematics(),
            ds::getWheelSpeeds,
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
            ds::tankDriveVolts,
            ds
        );
        
        
        this.driveSubsystem = ds;
        this.trajectory = trajectory;
    }

    public DriveRamsete(DriveSubsystem ds, String pathName) {
        this(ds, PathPlanner.loadPath(pathName, DriveConstants.maxVel, DriveConstants.maxAccel));
    }

    @Override
    public void initialize() {
        super.initialize();
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

}
