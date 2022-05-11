package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class CombinedCorrection {
    private ProfiledPIDController feedback;
    private SimpleMotorFeedforward feedforward;
    private correctionType type;

    public static enum correctionType{
        DISTANCE,
        VELOCITY
    }

    //super extended constructor, if this was C, this would've been named 'CombinedCorrectionEx' teehee C banter
    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback, TrapezoidProfile.Constraints constraints, correctionType type, double tolr){
        this.feedforward = feedforward;
        this.feedback = new ProfiledPIDController(feedback.getP(),feedback.getI(),feedback.getD(), constraints);
        this.type = type;
        feedback.setTolerance(tolr);
    }
    public CombinedCorrection(SimpleMotorFeedforward feedforward, ProfiledPIDController feedback, double tolr, correctionType type){
        this.feedforward = feedforward;
        this.feedback = feedback;
        this.type = type;
        feedback.setTolerance(tolr);
    }
    public CombinedCorrection(SimpleMotorFeedforward feedforward, ProfiledPIDController feedback, correctionType type){
        this.feedforward = feedforward;
        this.feedback = feedback;
        this.type = type;
    }
    //in this overload, type may be VELOCITY or DISTANCE
    public double getVoltage(double proccess, double setPoint){ 
        if(type == correctionType.DISTANCE){
        //it is now assumed that setPoint is distance to target
        double velocity = feedback.getSetpoint().velocity; 
        //now pid and ff are used to try and reach this velocity as close as possible since it is assumed that profilepid is correct in required velocity to reach target
        return feedback.calculate(proccess, new TrapezoidProfile.State(setPoint, 0)) +  feedforward.calculate(velocity); //assume that you want to reach setPoint with no remaining velocity
        }
        else
            return feedback.calculate(proccess, setPoint) + feedforward.calculate(setPoint); //acceleration
    }
    //in this overload, it is assumed that type = VELOCITY, exception thrown if not the case
    public double getVoltage(double proccess, double setVelocity, double setAcceleration) throws IllegalStateException{ 
        if(type == correctionType.DISTANCE)
            throw new IllegalStateException("Function overload of \'getVoltage\' with arguments (double process, double setVelocity, double setAcceleration is only allowed with CombinedCorrection.type = VELOCITY");
        else
            return feedback.calculate(proccess, setVelocity) + feedforward.calculate(setVelocity, setAcceleration);
    }
    public ProfiledPIDController accessPID(){
        return feedback;
    }
    public SimpleMotorFeedforward accessFeedForward(){
        return feedforward;
    }
}
