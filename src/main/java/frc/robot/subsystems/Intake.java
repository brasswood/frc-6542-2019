package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.OI;
import frc.robot.PIDList;
import frc.robot.PIDWidget;
import frc.robot.dashboard.Keys;

public class Intake extends Subsystem {

    private static Intake m_instance;
    private double kP, kI, kD, kF;
    private boolean m_enabled;

    private Spark intakeMotor = new Spark(OI.k_pwmIntakeMotor);

    private int multiplier = 1;

    NetworkTableEntry ntIntakeMotor = Shuffleboard.getTab(Keys.Tabs.tab_Subsystems).add(Keys.Widgets.widget_Intake, 0).getEntry();
    private Intake() {
        new PIDWidget("Intake PID", Shuffleboard.getTab(Keys.Tabs.tab_Subsystems)).addListener(new PIDUpdateListener());
    }

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    public void updatePID(double P, double I, double D, double F) {
        m_enabled = false;
        kP = P;
        kI = I;
        kD = D;
        kF = F;
    }
    @Override
    public void outputTelemetry() {
        ntIntakeMotor.setDouble(intakeMotor.get());
    }

    @Override
    public void init() {
        m_enabled = true;
    }

    @Override
    public void doRun() {
        if (m_enabled) {
            intakeMotor.set(OI.getInstance().getIntakeButton() * multiplier);
        } else {
            intakeMotor.set(0);
        }
    }

    private class PIDUpdateListener implements Consumer<PIDList>{
        public void accept(PIDList list) {
            updatePID(list.P, list.I, list.D, list.F);
        }
    }
}