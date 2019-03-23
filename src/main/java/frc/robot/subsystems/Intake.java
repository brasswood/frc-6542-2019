package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.OI;
import frc.robot.SeatMotorCounter;
import frc.robot.dashboard.PIDList;
import frc.robot.dashboard.PIDWidget;
import frc.robot.dashboard.WidgetProperties;
import frc.robot.dashboard.Keys;
import frc.robot.dashboard.LayoutBuilder;
import frc.robot.dashboard.NetworkTableHandle;

public class Intake extends Subsystem {

    private static Intake m_instance;
    private final double kP = 0, kI = 0, kD = 0, kF = 0;
    private boolean m_manualControl;
    private boolean m_isCalibrated = false;
    private IntakePosition m_position = IntakePosition.k_touchingGround;

    private Spark intakeMotor = new Spark(OI.k_pwmIntakeMotor);
    private int k_encoderPort = 0;

    // private final SeatMotorCounter counter = new SeatMotorCounter(k_encoderPort, () -> {return (int) Math.signum(intakeMotor.get());});
    private final Counter counter = new Counter(new DigitalInput(k_encoderPort));
    private final PIDController cont = new PIDController(kP, kI, kD, kF, counter, intakeMotor);

    private final double multiplier = 1;

    // SHUFFLEBOARD
    private final NetworkTableHandle ntOutput = new NetworkTableHandle();
    private final NetworkTableHandle ntCounter = new NetworkTableHandle();
    private final NetworkTableHandle ntSetpoint = new NetworkTableHandle();
    private final NetworkTableHandle ntError = new NetworkTableHandle();
    private final NetworkTableHandle ntIsCalibrated = new NetworkTableHandle();
    // END SHUFFLEBOARD

    private Intake() {
        //SHUFFLEBOARD
        new PIDWidget("Intake PID", Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), kP, kI, kD, kF).addListener(new PIDUpdateListener());
        cont.setPercentTolerance(5);
        WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties counter = new WidgetProperties(ntCounter, "Counter", BuiltInWidgets.kTextView, null, 0);
        WidgetProperties error = new WidgetProperties(ntError, "Error", null, 0);
        WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", null, 0);
        WidgetProperties isCalibrated = new WidgetProperties(ntIsCalibrated, "Calibrated", null, m_isCalibrated);
        WidgetProperties calibrate = new WidgetProperties(null, "Calibrate", notification -> {m_isCalibrated = (boolean) notification.value.getValue();}, false);

        ArrayList<WidgetProperties> positions = new ArrayList<WidgetProperties>();
        for (IntakePosition p : IntakePosition.values()) {
            positions.add(new WidgetProperties(p.handle, p.title, BuiltInWidgets.kToggleButton, new PositionListener(p), false));
        }
        
        LayoutBuilder.buildLayout("Intake Positions", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Control), 1, 1, positions.toArray(new WidgetProperties[positions.size()]));
        LayoutBuilder.buildLayout("Intake Debug", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Debug), 5, 5, new WidgetProperties[]{output, counter, setpoint, error});
        LayoutBuilder.buildLayout("Intake Calibration", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), 1, 1, new WidgetProperties[]{isCalibrated, calibrate});
        // END SHUFFLEBOARD
    }

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    public void updatePID(double P, double I, double D, double F) {
        cont.reset();
        cont.setPID(P, I, D, F);
    }
    @Override
    public void outputTelemetry() {
        ntOutput.setDouble(intakeMotor.get());
        ntCounter.setDouble(counter.get());
        // ntSetpoint.setDouble(cont.getSetpoint());
        // ntError.setDouble(cont.getError());
    }

    @Override
    public void init() {
    }

    @Override
    public void doRun() {

        // update state based on operator interface
        if (OI.getInstance().getIntakeManualSpeed() != 0) { // if we are manually controlling
            m_manualControl = true;
        } else if (OI.getInstance().getIntakeUpButton()) {
            m_manualControl = false;
            requestPosition(m_position.next());
        } else if (OI.getInstance().getIntakeDownButton()) {
            m_manualControl = false;
            requestPosition(m_position.previous());
        }

        // update motors based on state
        if (m_manualControl) {
            if (cont.isEnabled()) {
                cont.reset();
            }
            intakeMotor.set(OI.getInstance().getIntakeManualSpeed() * multiplier);
        } else {
            if (!cont.isEnabled()) {
                cont.enable();
            }
            System.out.println(cont.get());
            intakeMotor.pidWrite(cont.get());
        }
    }

    private void requestPosition(IntakePosition position) {
        cont.setSetpoint(position.setpoint);
        m_position = position;
    }

    public enum IntakePosition {
        k_touchingGround("Touch Ground", 0, 0), k_levelGround("Level", 0, 1), k_ballHold("Hold Ball", 0, 2), k_fullyUp("Fully Up", 0, 3);
        public String title;
        public double setpoint;
        public NetworkTableHandle handle;
        private int index;
        private static IntakePosition[] orderedValues = new IntakePosition[IntakePosition.values().length];
        private IntakePosition(String title, double setpoint, int index) {
            this.title = title;
            this.setpoint = setpoint;
            this.index = index;
            handle = new NetworkTableHandle();
            initialize();
        }

        private void initialize() {
            orderedValues[index] = this;
        }

        public IntakePosition next() {
            if (orderedValues.length > index) {
                return orderedValues[index+1];
            } else {
                return this;
            }
        }
        public IntakePosition previous() {
            if (index > 0) {
                return orderedValues[index-1];
            } else {
                return this;
            }
        }

    }
    private class PIDUpdateListener implements Consumer<PIDList>{
        public void accept(PIDList list) {
            updatePID(list.P, list.I, list.D, list.F);
        }
    }

    private class PositionListener implements Consumer<EntryNotification> {
        private IntakePosition requestedPosition;
        private PositionListener(IntakePosition requestedPosition) {
            this.requestedPosition = requestedPosition;
        }
        @Override
        public void accept(EntryNotification notification) {
            m_manualControl = false;
            requestPosition(requestedPosition);
        }
        
    }


}