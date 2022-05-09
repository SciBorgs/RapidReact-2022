package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class CombinedCorrection {
    private PIDController feedback;
    private SimpleMotorFeedforward feedforward;

    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback, double tolr){
        this.feedforward = feedforward;
        this.feedback = feedback;
        feedback.setTolerance(tolr);
    }
    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback){
        this.feedforward = feedforward;
        this.feedback = feedback;
    }
    
    public double getVoltage(double proccess, double setPoint){
        double resultFB = feedback.calculate(proccess, setPoint);
        double resultFF = feedforward.calculate(setPoint);
        return resultFB + resultFF;
    }
    public PIDController accessPID(){
        return feedback;
    }
    public SimpleMotorFeedforward accessFeedForward(){
        return feedforward;
    }
}
