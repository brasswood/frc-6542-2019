package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

// Wrapper class to allow for closed-loop current control using a PDP Channel
public class PidPDPChannel implements PIDSource {
    
    public final int k_channel;
    public final PowerDistributionPanel k_PDP;

    public PidPDPChannel(PowerDistributionPanel pdp, int channel) {
        k_PDP = pdp;
        k_channel = channel;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kRate;
    }

    @Override
    public double pidGet() {
        return k_PDP.getCurrent(k_channel);
    }

}