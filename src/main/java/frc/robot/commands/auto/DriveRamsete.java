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
    private boolean resetOdometry;

    public DriveRamsete(DriveSubsystem drive, Trajectory trajectory, boolean resetOdometry) {
        super(
            trajectory,
            drive::getPose,
            new RamseteController(),
            drive.getFeedforward(),
            drive.getKinematics(),
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
            drive::tankDriveVolts,
            drive
        );
        
        this.driveSubsystem = drive;
        this.trajectory = trajectory;
        this.resetOdometry = resetOdometry; 
        addRequirements(driveSubsystem);
    }

    public DriveRamsete(DriveSubsystem drive, String pathName, boolean resetOdometry) {
        this(drive, PathPlanner.loadPath(pathName, DriveConstants.maxVel, DriveConstants.maxAccel), resetOdometry);
    }

    @Override
    public void initialize() {
        super.initialize();
        if(resetOdometry) driveSubsystem.resetOdometry(trajectory.getInitialPose()); 
    }

    @Override 
    public void end(boolean interrupted) {
        driveSubsystem.tankDriveVolts(0, 0);
    }

}
