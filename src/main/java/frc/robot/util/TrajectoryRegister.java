package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * ALlows for multiple trajectories to be displayed on the field at once.
 *
 * <p>For use, set the field with TrajectoryRegister.setField2d().
 */
public class TrajectoryRegister {
  private static final List<String> slots =
      Collections.unmodifiableList(
          List.of(
              "traj-white",
              "traj-red",
              "traj-blue",
              "traj-yellow",
              "traj-green",
              "traj-orange",
              "traj-purple"));
  private static final Trajectory EMPTY_TRAJECTORY;
  private static Map<String, FieldObject2d> fieldObjects; // slot -> field object
  private static Map<String, SendableChooser<Trajectory>> choosers; // slot -> chooser
  private static Map<String, Trajectory> trajectories;

  static {
    fieldObjects = new HashMap<>();
    choosers = new HashMap<>();

    trajectories = new HashMap<>();
    List<String> names = Util.getPathPlannerPathNames();
    // pretty jank but this is a quick and dirty way to remove a trajectory from the screen
    EMPTY_TRAJECTORY =
        PathPlanner.loadPath(names.get(0), 0, 0)
            .transformBy(new Transform2d(new Translation2d(100, 100), new Rotation2d()));
    for (String name : names) {
      trajectories.put(
          name, PathPlanner.loadPath(name, DriveConstants.maxVel, DriveConstants.maxAccel));
    }

    putChoosers();
  }

  /**
   * Sets the field on which to display trajectories.
   *
   * @param field the desired field
   */
  public static void setField2d(Field2d field) {
    for (String slot : slots) {
      fieldObjects.put(slot, field.getObject(slot));
    }
  }

  private static void setTrajectory(String slot, Trajectory trajectory) {
    fieldObjects.get(slot).setTrajectory(trajectory);
  }

  private static SendableChooser<Trajectory> getTrajectoryChooser() {
    SendableChooser<Trajectory> chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", EMPTY_TRAJECTORY);
    for (Map.Entry<String, Trajectory> entry : trajectories.entrySet()) {
      chooser.addOption(entry.getKey(), entry.getValue());
    }
    return chooser;
  }

  /** Puts the trajectory choosers on Shuffleboard. */
  public static void putChoosers() {
    for (String slot : slots) {
      SendableChooser<Trajectory> chooser = getTrajectoryChooser();
      choosers.put(slot, chooser);
      SmartDashboard.putData(slot, chooser);
      Util.addSendableChooserListener(
          chooser, e -> setTrajectory(slot, choosers.get(slot).getSelected()));
    }
  }
}
