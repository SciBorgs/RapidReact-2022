package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleboardSubsystem {

    // Name convention here is that 'getting' refers to 'getting' something
    // from the Shuffleboard, and 'setting' refers to 'setting' something on
    // the Shuffleboard. Of course, this doesn't matter beacuse of the
    // overloaded bind(...) methods.
    private HashMap<String, HashMap<String, Consumer<Object>>> getterBindings;
    private HashMap<String, HashMap<String, Supplier<Object>>> setterBindings;
    private HashMap<String, HashMap<String, NetworkTableEntry>> networkTable;
    private HashMap<String, HashMap<String, Class<?>>> types;

    private static final HashMap<Class<?>, Function<NetworkTableValue, Object>> CASTING_TABLE;
    static {
        CASTING_TABLE = new HashMap<>();
        CASTING_TABLE.put(double.class,    NetworkTableValue::getDouble);
        CASTING_TABLE.put(Double.class,    NetworkTableValue::getDouble);
        CASTING_TABLE.put(boolean.class,   NetworkTableValue::getBoolean);
        CASTING_TABLE.put(Boolean.class,   NetworkTableValue::getBoolean);
        CASTING_TABLE.put(String.class,    NetworkTableValue::getString);
        CASTING_TABLE.put(double[].class,  NetworkTableValue::getDoubleArray);
        CASTING_TABLE.put(Double[].class,  NetworkTableValue::getDoubleArray);
        CASTING_TABLE.put(boolean[].class, NetworkTableValue::getBooleanArray);
        CASTING_TABLE.put(Boolean[].class, NetworkTableValue::getBooleanArray);
        CASTING_TABLE.put(String[].class,  NetworkTableValue::getStringArray);
    }

    public ShuffleboardSubsystem() {
        this.getterBindings = new HashMap<>();
        this.setterBindings = new HashMap<>();
        this.networkTable = new HashMap<>();
        this.types = new HashMap<>();
    }

    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Consumer<T> getter, T defaultValue) {
        this.registerTab(tab);
        getterBindings.get(tab).put(key, (Consumer<Object>) getter);
        networkTable  .get(tab).put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
        types         .get(tab).put(key, defaultValue.getClass());
    }

    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Supplier<T> setter, T defaultValue) {
        this.registerTab(tab);
        setterBindings.get(tab).put(key, (Supplier<Object>) setter);
        networkTable  .get(tab).put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
        types         .get(tab).put(key, defaultValue.getClass());
    }

    private void registerTab(String tab) {
        boolean getterNull = getterBindings.containsKey(tab);
        boolean setterNull = getterBindings.containsKey(tab);
        if (getterNull) this.getterBindings.put(tab, new HashMap<>());
        if (setterNull) this.setterBindings.put(tab, new HashMap<>());
        if (getterNull && setterNull) {
            this.networkTable.put(tab, new HashMap<>());
            this.types.put(tab, new HashMap<>());
        }
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

    public void update() {
        getterBindings.forEach(
            (tabName, getterBindingsForTab) -> {
                getterBindingsForTab.forEach(
                    (key, consumer) -> {
                        Class<?> type = this.types.get(tabName).get(key);
                        Object value = CASTING_TABLE.get(type).apply(networkTable.get(tabName).get(key).getValue());
                        consumer.accept(value);
                    }
                );
            }
        );
        setterBindings.forEach(
            (tabName, setterBindingsForTab) -> {
                setterBindingsForTab.forEach(
                    (key, supplier) -> {
                        Object value = supplier.get();
                        NetworkTableEntry entry = networkTable.get(tabName).get(key);
                        if (entry != null)
                            entry.setValue(value);
                        // else System.out.print("AYO WTF?????????????");
                    }
                );
            }
        );
    }
}
