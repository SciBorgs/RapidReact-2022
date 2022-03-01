package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
 
public class ShufflePID {
    private NetworkTableEntry pEntry, iEntry, dEntry, errDisplay;
    private ShuffleboardTab tab;
    private PID bind;

    public ShufflePID(String tabName, PID bindSet) {
        this(tabName, bindSet, "");
    }

    public ShufflePID(String tabName, PID bindSet, String distinguisher) {
        tab = Shuffleboard.getTab(tabName);
        pEntry = tab.add(distinguisher + " Proportional", bindSet.getP()).getEntry();
        iEntry = tab.add(distinguisher + " Integral", bindSet.getI()).getEntry();
        dEntry = tab.add(distinguisher + " Derivative", bindSet.getD()).getEntry();
        errDisplay = tab.add(distinguisher + " Error", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        bind = bindSet;
    }

    public void setProportional() {
        bind.setP(pEntry.getDouble(0.0));
    }

    public void setIntegral() {
        bind.setI(iEntry.getDouble(0.0));
    }

    public void setDerivative() {
        bind.setD(dEntry.getDouble(0.0));
    }
 
   public void updateError(double newError) {
        errDisplay.setDouble(newError);
   }

    public void update() {
        setProportional();
        setIntegral();
        setDerivative();
    }

} 
