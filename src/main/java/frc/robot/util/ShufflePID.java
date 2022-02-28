package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
 
public class ShufflePID {
    private NetworkTableEntry pEntry, iEntry, dEntry;
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

    public void update() {
        setProportional();
        setIntegral();
        setDerivative();
    }

} 
