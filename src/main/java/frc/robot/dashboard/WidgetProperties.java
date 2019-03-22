package frc.robot.dashboard;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public final class WidgetProperties {
    protected WidgetType type;
    protected Consumer<EntryNotification> listener;
    protected String title;
    protected Object defaultValue;
    protected NetworkTableHandle entryHandle;
    public WidgetProperties(NetworkTableHandle entryHandle, String title, WidgetType type, Consumer<EntryNotification> listener, Object defaultValue) {
        this.title = title;
        this.type = type;
        this.listener = listener;
        this.defaultValue = defaultValue;
        this.entryHandle = entryHandle;
    }

    public WidgetProperties(NetworkTableHandle entryHandle, String title, Consumer<EntryNotification> listener, Object defaultValue) {
        this(entryHandle, title, BuiltInWidgets.kTextView, listener, defaultValue);
    }

}