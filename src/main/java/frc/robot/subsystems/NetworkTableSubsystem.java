package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.networktables.EntryInfo;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.controllers.MovementController;
import frc.robot.util.PID;

/**
 * Class for allowing values to be continuously shared between the robot (i.e.
 * in a subsystem or command) and a NetworkTableInstance.
 * <p>
 * For example, if {@code DummySubsystem} is a subsystem with getter and setter 
 * methods {@code double getSpeed() } and {@code void setSpeed(double speed)},
 * then one may <em>bind</em> the speed to an entry named {@code speed} on the 
 * Shuffleboard network table with the following:
 * <pre>
 * // Robot.java
 * DummySubsystem        dummySubsystem        = new DummySubsystem();
 * NetworkTableSubsystem networkTableSubsystem = new NetworkTableSubsystem();
 *  ...
 * public void robotInit() {
 *      networkTableSubsystem.bind("demo", "set speed", dummySubsystem::getSpeed, 0.0);
 *      networkTableSubsystem.bind("demo", "speed", dummySubsystem::setSpeed, 0.0);
 * }
 *  ...
 * public void robotPeriodic() {
 *      networkTableSubsystem.update();
 * }
 * </pre>
 * Note that two different keys are provided for getting and setting. <b>Binding 
 * a Consumer and Supplier to the same network table entry will result in 
 * unexpected behavior.</b>
 * <p>
 * You can only create such bindings for network table entries in the Shuffleboard.
 * However you can still access values from across the entire NetworkTableInstance
 * with {@code Object getFromAddress(String address, Object defaultValue)}. To get
 * a list of addresses, you can call {@code NetworkTableSubsystem.toString()}. For
 * example:
 * <pre>
 * // Getting values directly from network table instance
 * System.out.println(networkTableSubsystem.toString());
 *  ...
 * 301989900    /FMSInfo/IsRedAlliance                             kBoolean        5635137  
 * 301989916    /Shuffleboard/localization/rH                      kDouble         5675653  
 * 301989925    /Shuffleboard/drive/.type                          kString         5679105  
 * 301989902    /FMSInfo/FMSControlData                            kDouble         5692787  
 * 301989896    /FMSInfo/EventName                                 kString         5634342 
 *  ...
 * System.out.println(networkTableSubsystem.getFromAddress("/FMSInfo/IsRedAlliance", false));
 *  ...
 * true
 * </pre>
 */
public class NetworkTableSubsystem {

    // Name convention here is that 'getting' refers to 'getting' something  //
    // from the Shuffleboard, and 'setting' refers to 'setting' something on //
    // the Shuffleboard. Of course, this doesn't matter beacuse of the       //
    // overloaded bind(...) methods.                                         //

    private HashMap<String, HashMap<String, Consumer<Object>>> getterBindings;
    private HashMap<String, HashMap<String, Supplier<Object>>> setterBindings;
    private HashMap<String, HashMap<String, NetworkTableEntry>> networkTable;

    private final NetworkTableInstance networkTableInstance;

    // NetworkTableEntry.getValue() returns a NetworkTableValue which cannot be
    // cast to Double, Boolean, etc. directly (or at least conveniently). This
    // allows us to cast with ease.
    private static final HashMap<NetworkTableType, Function<NetworkTableValue, Object>> CASTING_TABLE;
    static {
        CASTING_TABLE = new HashMap<>();
        CASTING_TABLE.put(NetworkTableType.kDouble,       NetworkTableValue::getDouble);
        CASTING_TABLE.put(NetworkTableType.kBoolean,      NetworkTableValue::getBoolean);
        CASTING_TABLE.put(NetworkTableType.kString,       NetworkTableValue::getString);
        CASTING_TABLE.put(NetworkTableType.kDoubleArray,  NetworkTableValue::getDoubleArray);
        CASTING_TABLE.put(NetworkTableType.kBooleanArray, NetworkTableValue::getBooleanArray);
        CASTING_TABLE.put(NetworkTableType.kStringArray,  NetworkTableValue::getStringArray);
    }

    /**
     * Creates a NetworkTableSubsystem using the default NetworkTableInstance.
     */
    public NetworkTableSubsystem() {
        this.getterBindings = new HashMap<>();
        this.setterBindings = new HashMap<>();
        this.networkTable = new HashMap<>();
        this.networkTableInstance = NetworkTableInstance.getDefault();
    }

    /**
     * Sets up a {@code Consumer} to take in the value of the appropriate
     * NetworkTableEntry at the specified tab. One may bind a subsystem's
     * setter method using the double colon operator (::):
     * <pre> bind(tab, key, subsystem::setvalue, defaultValue); </pre>
     * or with the lamba operator:
     * <pre> bind(tab, key, v -> { subsystem.setValue(v); }, defaultValue); </pre>
     * Calling this method will not overwrite, but rather chain the new binding.
     * @param <T> the type of the corresponding value
     * @param tab the Shuffleboard tab of the NetworkTableEntry
     * @param key the key of the NetworkTableEntry
     * @param getter the {@code Consumer} that accepts the value the NetworkTableEntry
     * @param defaultValue the default value to accept
     */
    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Consumer<T> getter, T defaultValue) {
        this.registerTab(tab);
        if (this.hasGetterBinding(tab, key)) {
            Consumer<Object> before = getterBindings.get(tab).get(key);
            getterBindings.get(tab).put(key, before.andThen((Consumer<Object>) getter));
        } else {
            getterBindings.get(tab).put(key, (Consumer<Object>) getter);
        }
        if (!this.hasAssignedNetworkTableEntry(tab, key))
            networkTable.get(tab).put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
    }

    /**
     * Sets up a {@code Supplier} to supply the value of the appropriate
     * NetworkTableEntry at the specified tab. One may bind a subsystem's
     * getter method using the double colon operator (::):
     * <pre> bind(tab, key, subsystem::getvalue, defaultValue); </pre>
     * or with the lamba operator:
     * <pre> bind(tab, key, () -> subsystem.getValue(v), defaultValue); </pre>
     * Calling this method will overwrite previous bindings.
     * @param <T> the type of the corresponding value
     * @param tab the Shuffleboard tab of the NetworkTableEntry
     * @param key the key of the NetworkTableEntry
     * @param setter the {@code Supplier} that supplies the value the NetworkTableEntry
     * @param defaultValue the default value to supply
     */
    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Supplier<T> setter, T defaultValue) {
        this.registerTab(tab);
        setterBindings.get(tab).put(key, (Supplier<Object>) setter);
        if (!this.hasAssignedNetworkTableEntry(tab, key))
            networkTable.get(tab).put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
    }

    /**
     * Gets the value of the NetworkTableEntry at the specified key address If 
     * the value is unassigned, the default value is returned.
     * <p>
     * Example:
     * <pre>
     * System.out.println(networkTableSubsystem.getFromAddress("/FMSInfo/IsRedAlliance", false));
     * </pre>
     * @param address the address of the network table entry
     * @param defaultValue the default value to return
     * @return the value at the specified address, or the default value
     */
    public Object getFromAddress(String address, Object defaultValue) {
        NetworkTableValue ntvalue = this.networkTableInstance.getEntry(address).getValue();
        if (ntvalue.getType() == NetworkTableType.kUnassigned) return defaultValue;
        return castToNormalObject(ntvalue);
    }

    /**
     * Removes all bindings for an entry in this NetworkTableSubsystem. This 
     * does not change the NetworkTableInstance itself.
     * @param tab the tab of the NetworkTableEntry
     * @param key the key of the NetworkTableEntry
     */
    public void removeBindings(String tab, String key) {
        setterBindings.get(tab).remove(key);
        getterBindings.get(tab).remove(key);
    }

    private void registerTab(String tab) {
        boolean getterNull = !getterBindings.containsKey(tab);
        boolean setterNull = !getterBindings.containsKey(tab);
        if (getterNull) this.getterBindings.put(tab, new HashMap<>());
        if (setterNull) this.setterBindings.put(tab, new HashMap<>());
        if (getterNull && setterNull)
            this.networkTable.put(tab, new HashMap<>());
    }

    public boolean hasGetterBinding(String tab, String key) {
        HashMap<String, Consumer<Object>> getterBindingsForTab = getterBindings.get(tab);
        if (getterBindingsForTab == null) return false;
        return getterBindingsForTab.containsKey(key);
    }

    public boolean hasSetterBinding(String tab, String key) {
        HashMap<String, Supplier<Object>> setterBindingsForTab = setterBindings.get(tab);
        if (setterBindingsForTab == null) return false;
        return setterBindingsForTab.containsKey(key);
    }

    public boolean hasBinding(String tab, String key) {
        return this.hasGetterBinding(tab, key) || this.hasSetterBinding(tab, key);
    }

    public boolean hasAssignedNetworkTableEntry(String tab, String key) {
        HashMap<String, NetworkTableEntry> networkTableForTab = networkTable.get(tab);
        if (networkTableForTab == null) return false;
        return networkTableForTab.containsKey(key);
    }

    private static Object castToNormalObject(NetworkTableValue ntvalue) {
        return CASTING_TABLE.get(ntvalue.getType()).apply(ntvalue);
    }

    /**
     * Call this method periodically in order to update all of the bindings.
     */
    public void update() {
        updateSetters();
        updateGetters();
    }

    private void updateGetters() {
        getterBindings.forEach(
            (tabName, getterBindingsForTab) -> {
                getterBindingsForTab.forEach(
                    (key, consumer) -> {
                        NetworkTableValue ntvalue = networkTable.get(tabName).get(key).getValue();
                        if (ntvalue.getType() != NetworkTableType.kUnassigned) {
                            Object value = castToNormalObject(networkTable.get(tabName).get(key).getValue());
                            consumer.accept(value);
                        }
                    }
                );
            }
        );
    }

    private void updateSetters() {
        setterBindings.forEach(
            (tabName, setterBindingsForTab) -> {
                setterBindingsForTab.forEach(
                    (key, supplier) -> {
                        Object value = supplier.get();
                        NetworkTableEntry entry = networkTable.get(tabName).get(key);
                        if (entry != null)
                            entry.setValue(value);
                    }
                );
            }
        );
    }

    /**
     * Gets the NetworkTableInstance of this NetworkTableSubsystem. This will
     * always return the same value as NetworkTableInstance.getDefault().
     * @return the NetworkTableInstance of this NetworkTableSubsystem
     */
    public NetworkTableInstance getNetworkTableInstance() {
        return this.networkTableInstance;
    }

    /**
     * Gets all of the NetworkTableEntries in the NetworkTableInstance of this
     * NetowrkTableSubsystem.
     * @return an array of NetworkTableEntries
     */
    public NetworkTableEntry[] getNetworkTableEntries() {
        return this.networkTableInstance.getEntries("", 0);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("NetworkTableSubsystem: ");
        for (NetworkTableEntry entry : this.getNetworkTableEntries()) {
            EntryInfo info = entry.getInfo();
            sb.append(
                String.format("%n%-12d %-50s %-15s %-9d",
                entry.getHandle(), info.name, info.type, info.last_change));
        }
        return sb.toString();
    }

    public static Supplier<Object> defaultSupplier(Object val) {
        return () -> val;
    }

    public static Consumer<Object> defaultConsumer() {
        return (val) -> {};
    }

    // convenience methods

    /**
     * Creates bindings in this NetworkTableSubsystem to track output, and,
     * optionally, track other info (error, integral, derivative) and set
     * PID coefficients.
     * <pre>
     * // Example use
     *  ntsubsystem.createControllerBindings(
     *      "test", "controller",
     *      pointController,
     *      0.0);
     * @param tab the tab to use
     * @param name the prefix to use when naming the network table entries
     * @param pid the PID controller
     * @param getInfo whether to track error, integral, and derivative
     * @param setCoeff whether to add entries to set coefficients
     */
    public void createPIDBindings(String tab, String name, PID pid, boolean getInfo, boolean setCoeff) {
        this.bind(tab, name + " output", () -> pid.getOutput(), 0.0);
        if (getInfo) {
            this.bind(tab, name + " err", pid::getError, 0.0);
            this.bind(tab, name + " int", pid::getIntegral, 0.0);
            this.bind(tab, name + " der", pid::getDerivative, 0.0);
        }
        if (setCoeff) {
            this.bind(tab, name + " set kP", pid::setP, pid.getP());
            this.bind(tab, name + " set kI", pid::setI, pid.getI());
            this.bind(tab, name + " set kD", pid::setD, pid.getD());
        }
    }

    /**
     * Create bindings in this NetworkTableSubsystem to track data and state
     * from a controller.
     * @param <T> the target type of the controller
     * @param <U> the type of the state enum of the controller
     * @param @param tab the tab to use
     * @param name the prefix to use when naming the network table entries
     * @param controller the controller
     * @param defaultValue the default value to use
     */
    public <T, U extends Enum<U>> void createControllerBindings(String tab, String name, MovementController<T, U> controller, U defaultValue) {
        this.bind(tab, name + " target value",  controller::getTarget, defaultValue);
        this.bind(tab, name + " current value", controller::getCurrentValue, defaultValue);
        this.bind(tab, name + " reached value", controller::atTarget, false);
        this.bind(tab, name + " controller state", () -> controller.getCurrentState().toString(), "");
        this.bind(tab, name + " controller fin",   controller::isFinished, false);
    }

    /**
     * Create bindings in this NetworkTableSubsystem to track data and state
     * from a controller. This method is overloaded to account for different
     * target types that cannot be sent through network tables (ex. Point) and
     * should be cast to a different type.
     * <pre>
     * // Example use
     *  ntsubsystem.createControllerBindings(
     *      "test", "controller",
     *      pathController,
     *      Point::toArray,
     *      new double[] {0.0, 0.0});
     * </pre>
     * @param <T> the target type of the controller
     * @param <U> the type of the state enum of the controller
     * @param <V> the type to cast the target type to
     * @param @param tab the tab to use
     * @param name the prefix to use when naming the network table entries
     * @param controller the controller
     * @param castingFunction the function to be used to cast from the target type to the network table type
     * @param defaultValue the default value to use
     */
    public <T, U extends Enum<U>, V> void createControllerBindings(String tab, String name, MovementController<T, U> controller, Function<T, V> castingFunction, V defaultValue) {
        this.bind(tab, name + " target value",  () -> castingFunction.apply(controller.getTarget()), defaultValue);
        this.bind(tab, name + " current value", () -> castingFunction.apply(controller.getCurrentValue()), defaultValue);
        this.bind(tab, name + " reached value", controller::atTarget, false);
        this.bind(tab, name + " controller state", () -> controller.getCurrentState().toString(), "");
        this.bind(tab, name + " controller fin",   controller::isFinished, false);
    }
}
