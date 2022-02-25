package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardSubsystem {

    // Name convention here is that 'getting' refers to 'getting' something
    // from the Shuffleboard, and 'setting' refers to 'setting' something on
    // the Shuffleboard. Of course, this doesn't matter if you use the regular
    // bind(...) function.
    private HashMap<String, HashMap<String, Consumer<Object>>> getterBindings;
    private HashMap<String, HashMap<String, Supplier<Object>>> setterBindings;

    public ShuffleboardSubsystem() {
        this.getterBindings = new HashMap<>();
        this.setterBindings = new HashMap<>();
    }

    @SuppressWarnings("unchecked")
    public <T> void bindGetter(String tab, String key, Consumer<T> getter) {
        HashMap<String, Consumer<Object>> getterBindingsForTab = getterBindings.get(tab);
        if (getterBindingsForTab == null)
            getterBindingsForTab = new HashMap<>();
        getterBindingsForTab.put(tab, (Consumer<Object>) getter);
    }

    @SuppressWarnings("unchecked")
    public <T> void bindSetter(String tab, String key, Supplier<T> setter) {
        HashMap<String, Supplier<Object>> setterBindingsForTab = setterBindings.get(tab);
        if (setterBindingsForTab == null)
            setterBindingsForTab = new HashMap<>();
        setterBindingsForTab.put(tab, (Supplier<Object>) setter);
    }

    @SuppressWarnings("unchecked")
    public <T, U> void bind(String tab, String key, Function<T, U> function) {
        if (function instanceof Consumer<?>) {
            this.bindGetter(tab, key, (Consumer<U>) function);
        } else if (function instanceof Supplier<?>) {
            this.bindSetter(tab, key, (Supplier<U>) function);
        } else {
            throw new IllegalArgumentException("Function is neither a Supplier or Consumer!");
        }
    }
}
