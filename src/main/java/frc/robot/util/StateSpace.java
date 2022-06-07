package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class StateSpace {
    private final double kV;
    private final double kA;
    private final LinearSystem<N1, N1, N1> systemPlant;
    private final KalmanFilter<N1, N1, N1> kalmanFilter;
    private final LinearQuadraticRegulator<N1, N1, N1> LQR;
    private final LinearSystemLoop<N1,N1,N1> loop;

    public StateSpace(double kV, double kA){
        this.kV = kV;
        this.kA = kA;
        systemPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
        kalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), systemPlant, VecBuilder.fill(3.0), VecBuilder.fill(0.1), 0.01);
        LQR = new LinearQuadraticRegulator<>(systemPlant, VecBuilder.fill(8.0), VecBuilder.fill(12), 0.02);
        loop = new LinearSystemLoop<>(systemPlant, LQR, kalmanFilter, 12.0, 0.012);
        
    }
    public void correct(double a){
        loop.correct(VecBuilder.fill(a));

    }
    public void predict(double a){
        loop.predict(a);
    }
    public double getU(int num){
        return loop.getU(num);
    }
    public void setNextR(double nextR){
        loop.setNextR(VecBuilder.fill(nextR));
    }
}
