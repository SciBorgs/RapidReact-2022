package frc.robot.autoProfile;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.util.Path;
import frc.robot.util.PathWeaverUtil;
import frc.robot.util.Point;
import frc.robot.util.Util;

import frc.robot.commands.ShootSequence;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommandGroup;

import frc.robot.commands.auto.FollowBallCommand;
import frc.robot.commands.auto.MoveUpToHubCommand;
import frc.robot.commands.auto.PurePursuitCommand;
import frc.robot.commands.hopper.StartHopperCommand;
import frc.robot.commands.test.MarkerCommand;
import frc.robot.commands.turret.AimTurretCommand;
import frc.robot.commands.turret.SetTurretCommand;

/**
 * Use this class for storing paths, points, etc. in the auto sequence.
 * Also allows us to easily adjust auto strategy.
 */
public class AutoProfile {
    public static final List<Point> PATH_TEST = Util.generateSinePath(4.0, 0.45, 1.35, 0.05).stream().map(p -> Util.add(p, Constants.POINT_HUB)).collect(Collectors.toList());
    public static final Point POINT_TEST = new Point(0, 0);

    public static final Point  STARTING_POINT = Constants.POINT_HUB;
    public static final double STARTING_HEADING = 0;

    public static final double TAXI_DISTANCE = 2.0;

    public static final String FIELD_STRING = 
    //   ___.___.___.___.___.___.___.___.___.___.___.___.___.___.___ 
    // Red  balls:  B D E / G I L / M N
    // Blue balls:  A C F / H J K / O P
    // Red  positions: 1 2 3 4
    // Blue positions: 5 6 7 8
        "/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///"
    + "\n//                    |_   F   [E]                   [M] //"
    + "\n//                 [G] |_                                //"
    + "\n                 H       |_ [4]       [D]                  "
    + "\n                       5   |     [3]                       "
    + "\n//                          =---=         C              //"
    + "\n//                   6     /   /   [2]                   //"
    + "\n//             [I]        =---=                          //"
    + "\n                        7      |_ [1]                      "
    + "\n                  J          8   |_     [B]                " 
    + "\n//                                 |_  A                 //"
    + "\n//  N                     K   [L]   |                    //"
    + "\n/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///";

    private static enum StageType { 
        INITIALIZE, // start hopper and intake
        TAXI,       // taxi to set distance from hub
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
                new StartHopperCommand(),
                new IntakeCommandGroup(),
                new MarkerCommand("Initializing")
            );
        }
    }

    private static class TaxiStage extends Stage {
        public TaxiStage() {
            super(StageType.TAXI);
            this.addCommands(
                new MoveUpToHubCommand(TAXI_DISTANCE)
            );
        }
    }

    private static class FollowStage extends Stage {
        public FollowStage() {
            super(StageType.FOLLOW);
            this.addCommands(
                new FollowBallCommand(),
                new MarkerCommand("Following")
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
                this.addCommands(new SetTurretCommand(desiredAngle - interceptAngle));
            }
            this.addCommands(
                new MarkerCommand("Transporting"),
                new PurePursuitCommand(path)
            );
        }
    }

    private static class ShootStage extends Stage {
        public ShootStage() {
            super(StageType.SHOOT);
            this.addCommands(new SequentialCommandGroup(
                new AimTurretCommand(),
                new ShootSequence(),
                new WaitCommand(1.155),
                new MarkerCommand("Shooting")
            ));
        }
    }

    private static Supplier<Stage[]> autoStrategy = () -> new Stage[] {};

    public static Stage[] getStagesFromStrategy(Strategy strategy) {
        TransportProfile profile = strategy.transportProfile;
        switch (strategy.type) {
            case Nothing:
                return new Stage[] {};
            case Taxi:
                return new Stage[] {
                    new InitializeStage(),
                    new TaxiStage()
                };
            case Shoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage()
                };
            case ShootMoveShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage()
                };
            case ShootMoveShootMoveShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(2), true),
                    new FollowStage(),
                    new ShootStage()
                };
            case ShootMoveShootMoveShootShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(2), true),
                    new FollowStage(),
                    new FollowStage(),
                    new ShootStage(),
                    new ShootStage()
                };
            case ShootMoveShootShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage(),
                    new ShootStage()
                };
            case ShootMoveShootShootMoveShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(2), true),
                    new FollowStage(),
                    new ShootStage()
                };
            case ShootMoveShootShootMoveShootShoot:
                return new Stage[] {
                    new InitializeStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(1), true),
                    new FollowStage(),
                    new ShootStage(),
                    new ShootStage(),
                    new TransportStage(profile.getStagePath(2), true),
                    new FollowStage(),
                    new ShootStage(),
                    new FollowStage(),
                    new ShootStage()
                };
            default:
                return new Stage[] {};
        }
    }

    private static String prevValue = "";
    public static void setStrategyByName(String strategyName) {
        if (strategyName.equals(prevValue)) return;

        prevValue = strategyName;
        Strategy strategy = Strategy.BY_NAME.get(strategyName.toUpperCase());
        if (strategy == null) return;

        System.out.println(strategy.toString());
        autoStrategy = () -> getStagesFromStrategy(strategy);
    }

    public static void setStrategy(Strategy strategy) {
        System.out.println(strategy.toString());
        autoStrategy = () -> getStagesFromStrategy(strategy);
    }

    public static Command getAutoCommand() {
        return new SequentialCommandGroup(autoStrategy.get());
    }

    public static SendableChooser<Strategy> getAutoChooser() {
        SendableChooser<Strategy> chooser = new SendableChooser<>();
        for (String name : Strategy.BY_NAME.keySet()) {
            chooser.addOption(name, Strategy.BY_NAME.get(name));
        }
        chooser.setDefaultOption("DEFAULT", Strategy.NOTHING);
        return chooser;
    }
}
