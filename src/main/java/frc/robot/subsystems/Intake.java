package frc.robot.subsystems;

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
    private double P, I, D, F;
    private boolean m_manualControl;
    private boolean m_isCalibrated = false;

    private final NetworkTableHandle ntOutput = new NetworkTableHandle();
    private final NetworkTableHandle ntCounter = new NetworkTableHandle();
    private final NetworkTableHandle ntSetpoint = new NetworkTableHandle();
    private final NetworkTableHandle ntError = new NetworkTableHandle();
    private final NetworkTableHandle ntIsCalibrated = new NetworkTableHandle();

    private Spark intakeMotor = new Spark(OI.k_pwmIntakeMotor);
    private int k_encoderPort = 0;

    // private final SeatMotorCounter counter = new SeatMotorCounter(k_encoderPort, () -> {return (int) Math.signum(intakeMotor.get());});
    private final Counter counter = new Counter(new DigitalInput(k_encoderPort));
    private final PIDController cont = new PIDController(kP, kI, kD, kF, counter, intakeMotor);

    private final double multiplier = 1;

    private Intake() {
        new PIDWidget("Intake PID", Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), kP, kI, kD, kF).addListener(new PIDUpdateListener());
        cont.setPercentTolerance(5);
        WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties counter = new WidgetProperties(ntCounter, "Counter", BuiltInWidgets.kTextView, null, 0);
        WidgetProperties fullyUp = new WidgetProperties(null, "Fully Up", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_fullyUp), false);
        WidgetProperties levelGround = new WidgetProperties(null, "Level Ground", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_levelGround), false);
        WidgetProperties touchingGround = new WidgetProperties(null, "Touching Ground", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_touchingGround), false);
        WidgetProperties ballHold = new WidgetProperties(null, "Ball Hold", BuiltInWidgets.kToggleButton, new SetpointListener(IntakePosition.k_ballHold), false);
        WidgetProperties error = new WidgetProperties(ntError, "Error", null, 0);
        WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", null, 0);
        WidgetProperties isCalibrated = new WidgetProperties(ntIsCalibrated, "Calibrated", null, m_isCalibrated);
        WidgetProperties calibrate = new WidgetProperties(null, "Calibrate", notification -> {m_isCalibrated = (boolean) notification.value.getValue();}, false);
        
        
        LayoutBuilder.buildLayout("Intake Positions", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Control), 1, 1, new WidgetProperties[]{fullyUp, levelGround, touchingGround, ballHold});
        LayoutBuilder.buildLayout("Intake Debug", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Debug), 5, 5, new WidgetProperties[]{output, counter, setpoint, error});
        LayoutBuilder.buildLayout("Intake Calibration", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), 1, 1, new WidgetProperties[]{isCalibrated, calibrate});
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
        ntCounter.setDouble(counter.get());
        // ntSetpoint.setDouble(cont.getSetpoint());
        // ntError.setDouble(cont.getError());
    }

    @Override
    public void init() {
    }

    @Override
    public void doRun() {

        if (OI.getInstance().getIntake() != 0) {
            m_manualControl = true;
        }

        if (m_manualControl) {
            if (cont.isEnabled()) {
                cont.reset();
            }
            intakeMotor.set(OI.getInstance().getIntake() * multiplier);
        } else {
            if (!cont.isEnabled()) {
                cont.enable();
            }
            System.out.println(cont.get());
            intakeMotor.pidWrite(cont.get());
        }
    }

    public enum IntakePosition {
        k_fullyUp(0.4), k_levelGround(0.6), k_touchingGround(0.63), k_ballHold(0.44);
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