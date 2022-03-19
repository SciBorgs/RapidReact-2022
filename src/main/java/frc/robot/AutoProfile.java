package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PurePursuitCommand;
import frc.robot.util.Path;
import frc.robot.util.PathWeaverUtil;
import frc.robot.util.Point;
import frc.robot.util.Util;

/**
 * Use this class for storing paths, points, etc. in the auto sequence.
 * Also allows us to easily adjust auto strategy.
 */
public class AutoProfile {

    public static final double SHOOTING_RADIUS_NEAR = 1.5;
    public static final double SHOOTING_RADIUS_FAR  = 3.3;

    public static final List<Point> PATH_TEST = Util.generateSinePath(4.0, 0.45, 1.35, 0.05);
    public static final Point POINT_TEST = new Point(0, 0);

    public static final Point  STARTING_POINT = new Point(0, 0);
    public static final double STARTING_HEADING = 0;

    public static final String FIELD_STRING = 
    //   ___.___.___.___.___.___.___.___.___.___.___.___.___.___.___ 
        "/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///"
    + "\n//                    |_   F    E                   M    //"
    + "\n//                  G  |_                              N //"
    + "\n                 H       |_  4         D                   "
    + "\n                       5   |      3                        "
    + "\n//                          =---=         C              //"
    + "\n//                   6     /   /    2                    //"
    + "\n//              I         =---=                          //"
    + "\n                        7      |_  1                       "
    + "\n                  J          8   |_      B                 " 
    + "\n// P                               |_  A                 //"
    + "\n//    O                   K    L    |                    //"
    + "\n/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///";

    // Alliance balls:  B D E / G I L / M N
    // Enemy balls:     A C F / H J K / O P

    // Alliance positions: 1 2 3 4
    // Enemy positions   : 5 6 7 8

    // Possible auto plays:
    // Scout  : 4 -> F -> E   (3-ball)
    //         -> D -> M/N    (5/6-ball)
    // Home   : 4/3 -> E -> D (3-ball)
    //         -> M/N         (4/5-ball)
    // Orbit  : 3 -> D        (2-ball)
    //         -> M/N         (3/4-ball)
    // Meerkat: 2 -> D        (2-ball)
    //         -> M/N         (3/4-ball)
    // Guard  : 1 -> B -> L   (3-ball)
    // Spy    : 4/3 -> E -> G (3-ball)
    // for more info just play with the paths in PathWeaver

    public static enum TransportProfile {
          SCOUT_LOW("Red-Scout-Home"), SCOUT_HIGH("Red-Scout-Home", "Red-Scout-Regress"),
           HOME_LOW("Red-Home"),        HOME_HIGH("Red-Home",    "Red-Regress"),
          ORBIT_LOW("Red-Orbit"),      ORBIT_HIGH("Red-Orbit",   "Red-Regress"),
        MEERKAT_LOW("Red-Meerkat"),  MEERKAT_HIGH("Red-Meerkat", "Red-Regress"),
        GUARD("Red-Guard"), SPY("Red-Spy"),
        TEST("Test-Peanut", "Test-Circle");
 
        public final int transportStages;
        private final String[] pathnames;

        private TransportProfile(String... pathnames) {
            this.transportStages = pathnames.length;
            this.pathnames = pathnames;
        }

        // stages are 1-indexed
        public String getStagePath(int i) {
            if (i > transportStages) throw new IllegalArgumentException();
            return pathnames[i - 1];
        }
    }

    private static enum StageType { 
        INITIALIZE, // start hopper and intake
        TRANSPORT,  // drive off tarmac, drive to balls
        FOLLOW,     // follow ball (if ball placement is not exact)
        SHOOT,      // shoot ball in place
    }

    private static class Stage extends ParallelCommandGroup {
        final StageType type;
        public Stage(StageType type) { this.type = type; }
    }

    private static class InitializeStage extends Stage {
        public InitializeStage() {
            super(StageType.INITIALIZE);
            this.addCommands(
                // StartHopperCommand(),
                // StartIntakeCommand(),
            );
        }
    }

    private static class FollowStage extends Stage {
        public FollowStage() {
            super(StageType.FOLLOW);
            this.addCommands(
                // FollowBallCommand()
            );
        }
    }

    private static class TransportStage extends Stage {
        public TransportStage(String pathName, boolean prepareTurret) {
            super(StageType.TRANSPORT);
            Path path = PathWeaverUtil.getPathByName(pathName);
            if (prepareTurret) {
                Point finalPoint = path.get(path.size() - 1);
                double desiredAngle = Util.angleToPoint(Util.subtract(finalPoint, Constants.POINT_HUB));
                double interceptAngle = Util.interceptionAngle(path, finalPoint, 0.1, 3);
                // this.addCommands(new TurnTurretCommand(desiredAngle - interceptAngle));
            }
            this.addCommands(new PurePursuitCommand(path));
        }
    }

    private static class ShootStage extends Stage {
        public ShootStage() {
            super(StageType.SHOOT);
            this.addCommands(new SequentialCommandGroup(
                // new AimTurretCommand(),
                // new ShootBallCommand()
            ));
        }
    }

    private static Stage[] autoStrategy = {};

    public static void fromTransportProfile(TransportProfile profile) {
        if (profile == TransportProfile.TEST) {
            autoStrategy = new Stage[] {
                new TransportStage(profile.getStagePath(1), true),
                new TransportStage(profile.getStagePath(2), true)
            };
        } else if (profile.transportStages == 1) {
            autoStrategy = new Stage[] {
                new InitializeStage(),
                new ShootStage(),
                new TransportStage(profile.getStagePath(1), true),
                new ShootStage(),
                new ShootStage(),
            };
        } else if (profile.transportStages == 2) {
            autoStrategy = new Stage[] {
                new InitializeStage(),
                new ShootStage(),
                new TransportStage(profile.getStagePath(1), true),
                new ShootStage(),
                new TransportStage(profile.getStagePath(2), true),
                new FollowStage(),
                new ShootStage(),
                new ShootStage(),
            };
        }
    }

    public static Command getAutoCommand() {
        return new SequentialCommandGroup(autoStrategy);
    }
}
