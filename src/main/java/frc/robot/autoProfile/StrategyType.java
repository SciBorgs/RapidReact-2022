package frc.robot.autoProfile;

public enum StrategyType {
    Nothing                             (0), 
    Taxi                                (0),
    Shoot                               (1),
    ShootMoveShoot                      (2),
    ShootMoveShootShoot                 (3),
    ShootMoveShootMoveShoot             (3),
    ShootMoveShootMoveShootShoot        (4),
    ShootMoveShootShootMoveShoot        (4),
    ShootMoveShootShootMoveShootShoot   (5);

    protected final int numBalls;

    private StrategyType(int numBalls) {
        this.numBalls = numBalls;
    }
}
