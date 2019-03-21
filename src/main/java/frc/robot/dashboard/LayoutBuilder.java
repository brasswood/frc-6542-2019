package frc.robot.dashboard;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class LayoutBuilder {

    public static NetworkTableEntry[] buildLayout(String title, LayoutType type, ShuffleboardContainer container, WidgetProperties[] subWidgets) {
        ShuffleboardLayout layout = container.getLayout(title, type);
        NetworkTableEntry[] ret = new NetworkTableEntry[subWidgets.length];
        for (WidgetProperties props : subWidgets) {
            NetworkTableEntry entry = layout.add(props.title, props.defaultValue).withWidget(props.type).getEntry();
            props.entryHandle.setEntry(entry);
            if (props.listener != null) {entry.addListener(props.listener, EntryListenerFlags.kUpdate);}
        }
        return ret;
    }
}