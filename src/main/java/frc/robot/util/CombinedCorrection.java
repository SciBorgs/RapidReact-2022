package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.shuffleboard.api.util.Time;

public class CombinedCorrection {
    private PIDController feedback;
    private SimpleMotorFeedforward feedforward;
    private TrapezoidProfile trapezoidal;
    private TrapezoidProfile.Constraints constraints;
    private long aux;

    //assumes to init profile like in docs
    //super extended constructor, if this was C, this would've been named 'CombinedCorrectionEx' teehee C banter
    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback, TrapezoidProfile.Constraints constraints, 
    TrapezoidProfile.State initalState, TrapezoidProfile.State goal, double tolr){
        this.feedforward = feedforward;
        this.feedback = feedback;
        feedback.setTolerance(tolr);
        //since updating trapeoid profile requires recreating it over and over
        this.constraints = constraints;
        this.goal = goal;

        trapezoidal = new TrapezoidProfile(constraints, goal, initalState);
        check();
    }

    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback, double tolr){
        this.feedforward = feedforward;
        this.feedback = feedback;
        feedback.setTolerance(tolr);
        check();
    }
    
    public CombinedCorrection(SimpleMotorFeedforward feedforward, PIDController feedback){
        this.feedforward = feedforward;
        this.feedback = feedback;
        check();
    }
    //may need to check more things in future, this is called in all constructor overloads
    private void check(){ 
        if(trapezoidal != null)
            aux = System.currentTimeMillis(); //start time for the trapazoidal thing
    }
    
    public double getVoltage(double proccess, double setPoint){ 
        if(trapezoidal == null)
            return feedback.calculate(proccess, setPoint) + feedforward.calculate(setPoint);
        //it is now assumed that setPoint is distance to target
        double velocity = (trapezoidal.calculate(((double)(System.currentTimeMillis() - aux)) / 1000)).velocity; //get 'wanted' velocity from trapezoid thing
        //now pid and ff are used to try and reach this velocity as close as possible since it is assumed that trap is correct in required veloicty to reach target
        double returnedFeedForward = feedforward.calculate(velocity);
        double returnedFeedBack = feedback.calculate(proccess, velocity);
        //update trap as specified in docs, this is the reason goal and constraints are stored
        trapezoidal = new TrapezoidProfile(constraints, new TrapezoidProfile.State(setPoint, 0), new TrapezoidProfile.State(proccess, velocity));
        return returnedFeedBack + returnedFeedForward; //(assumed that you want to reach your setpoint and have 0 remaining veloicty, also assumed that setPoint is in the same 'space' as inital)
    }
    public double getVoltage(double proccess, double setPoint, double setAcceleration){ 
        if(trapezoidal == null)
            return feedback.calculate(proccess, setPoint) + feedforward.calculate(setPoint, setAcceleration);
        //it is now assumed that setPoint is distance to target
        double velocity = (trapezoidal.calculate(((double)(System.currentTimeMillis() - aux)) / 1000)).velocity; //get 'wanted' velocity from trapezoid thing
        //now pid and ff are used to try and reach this velocity as close as possible since it is assumed that trap is correct in required veloicty to reach target
        double returnedFeedForward = feedforward.calculate(velocity, setAcceleration);
        double returnedFeedBack = feedback.calculate(proccess, velocity);
        //update trap as specified in docs, this is the reason goal and constraints are stored
        trapezoidal = new TrapezoidProfile(constraints, new TrapezoidProfile.State(setPoint, 0), new TrapezoidProfile.State(proccess, velocity));
        return returnedFeedBack + returnedFeedForward; //(assumed that you want to reach your setpoint and have 0 remaining veloicty, also assumed that setPoint is in the same 'space' as inital)
    }
    public PIDController accessPID(){
        return feedback;
    }
    public SimpleMotorFeedforward accessFeedForward(){
        return feedforward;
    }
}
