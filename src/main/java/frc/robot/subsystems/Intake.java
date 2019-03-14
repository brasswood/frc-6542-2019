package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Intake extends Subsystem {

    private static Intake m_instance;

    private Spark intakeMotor = new Spark(OI.k_pwmIntakeMotor);

    private int multiplier = 1;

    NetworkTableEntry ntIntakeMotor = Shuffleboard.getTab(Keys.Tabs.tab_Subsystems).add(Keys.Widgets.widget_Intake, 0).getEntry();
    private Intake() {

    }

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    @Override
    public void outputTelemetry() {
        ntIntakeMotor.setDouble(intakeMotor.get());
    }

    @Override
    public void init() {

    }

    @Override
    public void doRun() {
        intakeMotor.set(OI.getInstance().getIntakeButton() * multiplier);
    }
}