package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class CombinedCorrection<PIDModel> {
    private PIDModel feedback;
    private SimpleMotorFeedforward feedforward;

    public static enum correctionType{
        DISTANCE,
        VELOCITY
    }

    private correctionType type;

    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDModel feedback, double tolr){
        this.feedforward = feedforward;
        this.feedback = feedback;
        check(feedback, tolr);
    }
    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDModel feedback){
        this.feedforward = feedforward;
        this.feedback = feedback;
        check(feedback);
    }
    private void check(PIDModel checkback, double tolr) throws IllegalStateException{
        if(checkback instanceof PIDController){
            type = correctionType.VELOCITY;
            ((PIDController)feedback).setTolerance(tolr);
        }
        else if(checkback instanceof ProfiledPIDController){
            type = correctionType.DISTANCE;
            ((ProfiledPIDController)feedback).setTolerance(tolr);
        }
        else
            throw new IllegalStateException("<PIDModel> of CombinedCorrection is not PIDController or ProfiledPIDController. This is not allowed.");
    }
    private void check(PIDModel checkback){
        if(checkback instanceof PIDController)
            type = correctionType.VELOCITY;
        else if(checkback instanceof ProfiledPIDController)
            type = correctionType.DISTANCE;
        else
            throw new IllegalStateException("<PIDModel> of CombinedCorrection is not PIDController or ProfiledPIDController. This is not allowed.");
    }

    public double getVoltage(double proccess, double setPoint){ 
        if(type == correctionType.DISTANCE){
        //it is now assumed that setPoint is distance to target
        double velocity = ((ProfiledPIDController)feedback).getSetpoint().velocity; 
        //now pid and ff are used to try and reach this velocity as close as possible since it is assumed that profilepid is correct in required velocity to reach target
        return ((ProfiledPIDController)feedback).calculate(proccess, new TrapezoidProfile.State(setPoint, 0)) +  feedforward.calculate(velocity); //assume that you want to reach setPoint with no remaining velocity
        }
        else
            return ((PIDController)feedback).calculate(proccess, setPoint) + feedforward.calculate(setPoint); //acceleration
    }

    public double getVoltage(double proccess, double setPoint, double setAcceleration){ 
        if(type == correctionType.DISTANCE){
        //it is now assumed that setPoint is distance to target
        double velocity = ((ProfiledPIDController)feedback).getSetpoint().velocity; 
        //now pid and ff are used to try and reach this velocity as close as possible since it is assumed that profilepid is correct in required velocity to reach target
        return ((ProfiledPIDController)feedback).calculate(proccess, new TrapezoidProfile.State(setPoint, 0)) +  feedforward.calculate(velocity, setAcceleration); //assume that you want to reach setPoint with no remaining velocity
        }
        else
        return ((PIDController)feedback).calculate(proccess, setPoint) + feedforward.calculate(setPoint, setAcceleration); //acceleration
    }
    public PIDModel accessPID(){
        return feedback;
    }
    public SimpleMotorFeedforward accessFeedForward(){
        return feedforward;
    }
}
