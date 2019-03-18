package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class PIDWidget {
    private ShuffleboardLayout m_layout;
    private NetworkTableEntry nt_P, nt_I, nt_D, nt_F, nt_update;
    private List<Consumer<PIDList>> m_listeners = new ArrayList();
    private boolean m_toggle;
    
    public PIDWidget(String name, ShuffleboardContainer container) {
        m_layout = container.getLayout(name, BuiltInLayouts.kList);
        nt_P = m_layout.add("P", 0).getEntry();
        nt_I = m_layout.add("I", 0).getEntry();
        nt_D = m_layout.add("D", 0).getEntry();
        nt_F = m_layout.add("F", 0).getEntry();
        nt_update = m_layout.add("Update PID", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        nt_update.addListener(
    }

    public double getP() {
        NetworkTableEntry
    }

    public void addListener(Consumer<PIDList> pidList) {
        m_listeners.add(pidList);
    }

    private class UpdateButtonListener implements Consumer<EntryNotification> {
        public void accept(EntryNotification notification) {
            
        }
    }
}