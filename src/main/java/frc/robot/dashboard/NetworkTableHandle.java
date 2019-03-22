package frc.robot.dashboard;

import edu.wpi.first.networktables.NetworkTableEntry;

public class NetworkTableHandle {
    private NetworkTableEntry m_entry;

    protected void setEntry(NetworkTableEntry entry) {
        m_entry = entry;
    }

    public void setDouble(double value){
            m_entry.setDouble(value);
    }

    public void setBoolean(boolean value){
        m_entry.setBoolean(value);

    }

    public void setString(String value){
        m_entry.setString(value);
    }

    public void setValue(Object value){
        m_entry.setValue(value);
    }

    public double getDouble(double defaultValue){
        return m_entry.getDouble(defaultValue);
    }
}