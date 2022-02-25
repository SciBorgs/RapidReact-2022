package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleboardSubsystem {

    // Name convention here is that 'getting' refers to 'getting' something
    // from the Shuffleboard, and 'setting' refers to 'setting' something on
    // the Shuffleboard. Of course, this doesn't matter beacuse of the
    // overloaded bind(...) methods.
    private HashMap<String, HashMap<String, Consumer<Object>>> getterBindings;
    private HashMap<String, HashMap<String, Supplier<Object>>> setterBindings;
    private HashMap<String, HashMap<String, NetworkTableEntry>> networkTable;

    public ShuffleboardSubsystem() {
        this.getterBindings = new HashMap<>();
        this.setterBindings = new HashMap<>();
        this.networkTable = new HashMap<>();
    }

    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Consumer<T> getter, Object defaultValue) {
        HashMap<String, Consumer<Object>> getterBindingsForTab = getterBindings.get(tab);
        HashMap<String, NetworkTableEntry> networkTableForTab = networkTable.get(tab);
        if (getterBindingsForTab == null) {
            getterBindingsForTab = new HashMap<>();
            networkTableForTab = new HashMap<>();
            this.getterBindings.put(tab, getterBindingsForTab);
            this.networkTable.put(tab, networkTableForTab);
        }
        getterBindingsForTab.put(key, (Consumer<Object>) getter);
        networkTableForTab.put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
    }

    @SuppressWarnings("unchecked")
    public <T> void bind(String tab, String key, Supplier<T> setter, Object defaultValue) {
        HashMap<String, Supplier<Object>> setterBindingsForTab = setterBindings.get(tab);
        HashMap<String, NetworkTableEntry> networkTableForTab = networkTable.get(tab);
        if (setterBindingsForTab == null) {
            setterBindingsForTab = new HashMap<>();
            networkTableForTab = new HashMap<>();
            this.setterBindings.put(tab, setterBindingsForTab);
            this.networkTable.put(tab, networkTableForTab);
        }
        setterBindingsForTab.put(key, (Supplier<Object>) setter);
        networkTableForTab.put(key, Shuffleboard.getTab(tab).add(key, defaultValue).getEntry());
    }

    @SuppressWarnings("unchecked")
    public <T, U> void bind(String tab, String key, Function<T, U> function, Object defaultValue) {
        if (function instanceof Consumer<?>) {
            this.bind(tab, key, (Consumer<U>) function, defaultValue);
        } else if (function instanceof Supplier<?>) {
            this.bind(tab, key, (Supplier<U>) function, defaultValue);
        } else {
            throw new IllegalArgumentException("Function is neither a Supplier or Consumer!");
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
                        Object value = networkTable.get(tabName).get(key).getValue();
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
                        System.out.printf("(%s, %s), %s%n", tabName, key, value);
                        NetworkTableEntry entry = networkTable.get(tabName).get(key);
                        if (entry != null)
                            entry.setValue(value);
                    }
                );
            }
        );
    }
}
