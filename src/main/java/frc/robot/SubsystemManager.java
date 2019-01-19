package frc.robot;

import java.util.List;

import frc.robot.subsystems.*;

public class SubsystemManager {

    // Team 254 is really good at OOP
    private final List<Subsystem> m_allSubsystems;
    
    public SubsystemManager(List<Subsystem> allSubsystems) {
        m_allSubsystems = allSubsystems;
    }

    public void outputToSmartDashboard() {
        m_allSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void run() {
        m_allSubsystems.forEach((s) -> s.doRun());
    }

}