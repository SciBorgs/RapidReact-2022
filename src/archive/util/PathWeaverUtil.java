package frc.robot.util;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * I made this class so we wouldnt have to write out {@code java.nio.Path} and
 * {@code frc.robot.util.Path} when dealing with both of them in the same file
 * (of course excluding this one).
 * <p>
 * I know, it's stupid.
 */
public class PathWeaverUtil {
    private static HashMap<String, frc.robot.util.Path> stored = new HashMap<>();

    public static Trajectory getTrajectoryByName(String trajectoryJSON) {
        Trajectory trajectory;
        try {
            java.nio.file.Path trajectoryPath 
                = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            trajectory = null;
        }
        return trajectory;
    }

    public static frc.robot.util.Path getPathByName(String name) {
        if (stored.keySet().contains(name)) return stored.get(name);
        Trajectory j = getTrajectoryByName("paths/output/" + name + ".wpilib.json");
        List<Point> points = j.getStates().stream()
                     .map(state -> state.poseMeters)
                     .map(pose -> new Point(pose.getX(), pose.getY()))
                     .collect(Collectors.toList());
        frc.robot.util.Path path = Util.reparameterize(points, 0.05);
        stored.put(name, path);
        return path;
    }
}
