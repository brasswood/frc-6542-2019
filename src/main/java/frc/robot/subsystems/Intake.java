package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.OI;
import frc.robot.dashboard.PIDList;
import frc.robot.dashboard.PIDWidget;
import frc.robot.dashboard.WidgetProperties;
import frc.robot.dashboard.Keys;
import frc.robot.dashboard.LayoutBuilder;
import frc.robot.dashboard.NetworkTableHandle;

public class Intake extends Subsystem {

    private static Intake m_instance;
    private final double kP = 0, kI = 0, kD = 0, kF = 0;
    private double P, I, D, F;
    private boolean m_manualControl;

    private final NetworkTableHandle ntOutput = new NetworkTableHandle();
    private final NetworkTableHandle ntPotentiometer = new NetworkTableHandle();
    private final NetworkTableHandle ntSetpoint = new NetworkTableHandle();
    private final NetworkTableHandle ntError = new NetworkTableHandle();

    private Spark intakeMotor = new Spark(OI.k_pwmIntakeMotor);
    private AnalogPotentiometer potentiometer = new AnalogPotentiometer(OI.k_intakePotPort);

    private final PIDController cont = new PIDController(kP, kI, kD, kF, potentiometer, intakeMotor);

    private final double multiplier = 1;

    private Intake() {
        new PIDWidget("Intake PID", Shuffleboard.getTab(Keys.Tabs.tab_Subsystems), kP, kI, kD, kF).addListener(new PIDUpdateListener());
        cont.setPercentTolerance(5);
        WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties potentiometer = new WidgetProperties(ntPotentiometer, "Potentiometer", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties position1 = new WidgetProperties(null, "Position 1", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_position3), false);
        WidgetProperties position2 = new WidgetProperties(null, "Position 2", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_position2), false);
        WidgetProperties position3 = new WidgetProperties(null, "Position 3", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_position3), false);
        WidgetProperties error = new WidgetProperties(ntError, "Error", null, 0);
        WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", null, 0);
        WidgetProperties[] widgets = {output, potentiometer, position1, position2, position3, setpoint, error};
        LayoutBuilder.buildLayout("Intake", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Subsystems), widgets);
    }

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    public void updatePID(double P, double I, double D, double F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
        cont.reset();
        cont.setPID(P, I, D, F);
    }
    @Override
    public void outputTelemetry() {
        ntOutput.setDouble(intakeMotor.get());
        ntPotentiometer.setDouble(potentiometer.get());
        ntSetpoint.setDouble(cont.getSetpoint());
        ntError.setDouble(cont.getError());
    }

    @Override
    public void init() {
    }

    @Override
    public void doRun() {

        if (OI.getInstance().getIntakeButton() != 0) {
            m_manualControl = true;
        }

        if (m_manualControl) {
            if (cont.isEnabled()) {
                cont.reset();
            }
            intakeMotor.set(OI.getInstance().getIntakeButton() * multiplier);
        } else {
            if (!cont.isEnabled()) {
                cont.enable();
            }
        }
    }

    public enum IntakePosition {
        k_position1(0), k_position2(1), k_position3(2);
        public double setpoint;
        private IntakePosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    private class PIDUpdateListener implements Consumer<PIDList>{
        public void accept(PIDList list) {
            updatePID(list.P, list.I, list.D, list.F);
        }
    }

    private class SetpointListener implements Consumer<EntryNotification> {
        private IntakePosition requestedPosition;
        private SetpointListener(IntakePosition requestedPosition) {
            this.requestedPosition = requestedPosition;
        }
        @Override
        public void accept(EntryNotification notification) {
            cont.setSetpoint(requestedPosition.setpoint);
            m_manualControl = false;
        }
        
    }
}