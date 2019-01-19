package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class Power extends Subsystem {
    private static Power m_instance;
    private PowerDistributionPanel m_PDP = new PowerDistributionPanel(OI.k_canPDPID);
    private double[] m_current = new double[16];

    private Power() {

    }

    public static Power getInstance() {
        if (m_instance == null) {
            m_instance = new Power();
        }
        return m_instance;
    }

    private void update() {
        for (int ch = 0; ch < 16; ch++) {
            m_current[ch] = m_PDP.getCurrent(ch);
        }
    }
    
    public void outputTelemetry() {
        SmartDashboard.putNumber("current[0]", m_current[0]);
        SmartDashboard.putNumberArray("currentDraw", m_current);
    }

    public void doRun() {
        update();
    }
}