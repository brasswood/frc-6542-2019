package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class PIDWidget {
    private ShuffleboardLayout m_layout;
    private final NetworkTableEntry nt_P, nt_I, nt_D, nt_F, nt_update;
    private final double m_defaultP, m_defaultI, m_defaultD, m_defaultF;
    private List<Consumer<PIDList>> m_listeners = new ArrayList<Consumer<PIDList>>();
    
    public PIDWidget(String name, ShuffleboardContainer container, double defaultP, double defaultI, double defaultD, double defaultF) {
        m_layout = container.getLayout(name, BuiltInLayouts.kList);
        m_defaultP = defaultP;
        m_defaultI = defaultI;
        m_defaultD = defaultD;
        m_defaultF = defaultF;
        nt_P = m_layout.add("P", defaultP).getEntry();
        nt_I = m_layout.add("I", defaultI).getEntry();
        nt_D = m_layout.add("D", defaultD).getEntry();
        nt_F = m_layout.add("F", defaultF).getEntry();
        nt_update = m_layout.add("Update PID", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        nt_update.addListener(new UpdateButtonListener(), EntryListenerFlags.kUpdate);
    }

    public PIDWidget(String name, ShuffleboardContainer container) {
        this(name, container, 0d, 0d, 0d, 0d);
    }

    public void addListener(Consumer<PIDList> pidList) {
        m_listeners.add(pidList);
    }

    public void setPID(PIDList list) {
        nt_P.setDouble(list.P);
        nt_I.setDouble(list.I);
        nt_D.setDouble(list.D);
        nt_F.setDouble(list.F);
    }

    public PIDList getPID() {
        return new PIDList(nt_P.getDouble(m_defaultP), nt_I.getDouble(m_defaultI), nt_D.getDouble(m_defaultD), nt_F.getDouble(m_defaultF));
    }
    
    private class UpdateButtonListener implements Consumer<EntryNotification> {
        public void accept(EntryNotification notification) {
            PIDList list = new PIDList(nt_P.getDouble(0), nt_I.getDouble(0), nt_D.getDouble(0), nt_F.getDouble(0));
            for (Consumer<PIDList> listener : m_listeners) {
                listener.accept(list);
            }
        }
    }
}