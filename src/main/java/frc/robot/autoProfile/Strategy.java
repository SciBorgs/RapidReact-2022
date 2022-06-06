package frc.robot.autoProfile;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public enum Strategy {
    // No movement
    NOTHING         (TransportProfile.R_0,      StrategyType.Nothing),
    SHOOT_IN_PLACE  (TransportProfile.R_0,      StrategyType.Shoot),

    // Taxi
    TAXI            (TransportProfile.R_0,      StrategyType.Taxi),

    // Start at pos 1
    S_1_2BALL_A     (TransportProfile.R_1B,     StrategyType.ShootMoveShoot),
    S_1_2BALL_B     (TransportProfile.R_1L,     StrategyType.ShootMoveShoot),
    S_1_3BALL       (TransportProfile.R_1BL,    StrategyType.ShootMoveShootShoot),

    // Start at pos 2
    S_2_2BALL_A     (TransportProfile.R_2B,     StrategyType.ShootMoveShoot),
    S_2_2BALL_B     (TransportProfile.R_2D,     StrategyType.ShootMoveShoot),

    // Start at pos 3
    S_3_2BALL       (TransportProfile.R_3D,     StrategyType.ShootMoveShoot),
    S_3_3BALL_A     (TransportProfile.R_3DE,    StrategyType.ShootMoveShootShoot),
    S_3_3BALL_B     (TransportProfile.R_3DM,    StrategyType.ShootMoveShootMoveShoot),
    S_3_4BALL       (TransportProfile.R_3DM,    StrategyType.ShootMoveShootMoveShootShoot),

    // Start at pos 4
    S_4_2BALL_A     (TransportProfile.R_4E,     StrategyType.ShootMoveShoot),
    S_4_2BALL_B     (TransportProfile.R_4G,     StrategyType.ShootMoveShoot),
    S_4_3BALL_A     (TransportProfile.R_4ED,    StrategyType.ShootMoveShootShoot),
    S_4_3BALL_B     (TransportProfile.R_4EG,    StrategyType.ShootMoveShootShoot),
    S_4_3BALL_C     (TransportProfile.R_4EM,    StrategyType.ShootMoveShootMoveShoot),
    S_4_4BALL_A     (TransportProfile.R_4EM,    StrategyType.ShootMoveShootMoveShootShoot),
    S_4_4BALL_B     (TransportProfile.R_4EDM,   StrategyType.ShootMoveShootShootMoveShoot),
    S_4_5BALL       (TransportProfile.R_4EDM,   StrategyType.ShootMoveShootShootMoveShootShoot);

    protected static final Map<String, Strategy> BY_NAME;
    static {
        BY_NAME = new HashMap<>();
        for (String name : Set.of(
            "NOTHING", "SHOOT_IN_PLACE", "TAXI",
            "S_1_2BALL_A", "S_1_2BALL_B", "S_1_3BALL",
            "S_2_2BALL_A", "S_2_2BALL_B",
            "S_3_2BALL", "S_3_3BALL_A", "S_3_3BALL_B", "S_3_4BALL",
            "S_4_2BALL_A", "S_4_2BALL_B", "S_4_3BALL_A", "S_4_3BALL_B", "S_4_3BALL_C", "S_4_4BALL_A", "S_4_4BALL_B", "S_4_5BALL"
        )) BY_NAME.put(name, valueOf(name));
    }
    
    protected final TransportProfile transportProfile;
    protected final StrategyType type;

    private Strategy(TransportProfile transportProfile, StrategyType type) {
        this.transportProfile = transportProfile;
        this.type = type;
    }
}
