package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Power extends Subsystem {
    private static Power m_instance;
    private final int k_numOfChannels = 16;
    private PowerDistributionPanel m_PDP = new PowerDistributionPanel(OI.k_canPDPID);
    private double[] m_current = new double[k_numOfChannels];

    private Power() {

    }

    public static Power getInstance() {
        if (m_instance == null) {
            m_instance = new Power();
        }
        return m_instance;
    }

    public void init() {
        
    }

    private void update() {
        for (int ch = 0; ch < k_numOfChannels; ch++) {
            m_current[ch] = m_PDP.getCurrent(ch);
        }
    }
    
    public void outputTelemetry() {
        SmartDashboard.putData(Keys.pdp, m_PDP);
    }

    public void doRun() {
        update();
    }

    public double getCurrent(int channel) {
        return m_PDP.getCurrent(channel);
    }
}