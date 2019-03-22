package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.PIDList;
import frc.robot.dashboard.PIDWidget;
import frc.robot.dashboard.WidgetProperties;
import frc.robot.dashboard.Keys;
import frc.robot.dashboard.LayoutBuilder;
import frc.robot.dashboard.NetworkTableHandle;

public class Elevator extends Subsystem {

  private WPI_TalonSRX m_elevatorMotor = new WPI_TalonSRX(OI.k_canElevatorMotor);
  private static Elevator m_instance;
  // private Timer m_holdTimer = new Timer();
  private boolean m_timerStarted = false;
  // private ElevatorState m_state = ElevatorState.k_holding;
  private boolean m_calibrated;
  private boolean m_manualControl = true;
<<<<<<< Updated upstream
  private boolean m_holding = true;
=======
  private boolean m_holding = true;;
>>>>>>> Stashed changes
  private double m_currentHeight;

  private final double k_upSpeed = 1;
  private final double k_downSpeed = -0.5;
  private final double k_holdSpeed = 0.1;
  private final double k_maxCurrent = 70;
  private final double k_maxStallTime = 2.5;

  private final boolean k_sensorPhase = true;
  private final boolean k_isInverted = false;

  private final NetworkTableHandle ntOutput = new NetworkTableHandle();
  private final NetworkTableHandle ntPosition = new NetworkTableHandle();
  private final NetworkTableHandle ntSetpoint = new NetworkTableHandle();
  private final NetworkTableHandle ntError = new NetworkTableHandle();
  private final NetworkTableHandle ntCalibrated = new NetworkTableHandle();

<<<<<<< Updated upstream

  private final double kP = 0, kI = 0, kD = 0, kF = 0;
=======
  private static final PositionListener positionListener = new PositionListener();

  private final double kP, kI, kD, kF;

  BRAINSTORM ELEVATOR
  POSITIONS
  HAVE PID IMPLEMENTATION READY
>>>>>>> Stashed changes

  private Elevator() {
    ShuffleboardTab tab_intake = Shuffleboard.getTab(Keys.Tabs.tab_Subsystems);
    new PIDWidget("Elevator PID", tab_intake, kP, kI, kD, kF).addListener(new PIDUpdateListener());
    WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
    WidgetProperties position = new WidgetProperties(ntPosition, "Position", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties error = new WidgetProperties(ntError, "Error", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties calibrated = new WidgetProperties(ntCalibrated, "CALIBRATED", BuiltInWidgets.kBooleanBox, null, false);
    WidgetProperties calibrateButton = new WidgetProperties(null, "CALIBRATE", BuiltInWidgets.kToggleButton, new CalibrationListener(), false);

    ArrayList<WidgetProperties> positionsArray = new ArrayList<WidgetProperties>();
    for (ElevatorPosition p : ElevatorPosition.values()) {
      positionsArray.add(p.props);
    }
    LayoutBuilder.buildLayout("Positions", BuiltInLayouts.kList, tab_intake, (WidgetProperties[]) positionsArray.toArray());
<<<<<<< Updated upstream
    WidgetProperties[] elevatorWidgetArray = {output, position, setpoint, error, calibrated, calibrateButton};
    LayoutBuilder.buildLayout("Elevator", BuiltInLayouts.kGrid, tab_intake, elevatorWidgetArray);
=======
>>>>>>> Stashed changes
  }

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void init() {
    m_currentHeight = 0;
  }

  @Override
  public void doRun() {
    // Poll controls to set manual mode
    if (OI.getInstance().getElevatorUpButton() || OI.getInstance().getElevatorDownButton()) {
      setManual(true);
    }
    if (m_manualControl) {
      if (OI.getInstance().getElevatorUpButton()) {
<<<<<<< Updated upstream
        m_holding = false;
        m_elevatorMotor.set(ControlMode.PercentOutput, k_upSpeed);
      } else if (OI.getInstance().getElevatorDownButton()) {
        m_holding = false;
        m_elevatorMotor.set(ControlMode.PercentOutput, k_downSpeed);
      } else {
        m_holding = true;
      }
    }

    if (m_holding) {
      m_elevatorMotor.set(ControlMode.PercentOutput, k_holdSpeed);
    }
=======
        m_elevatorMotor.set(ControlMode.PercentOutput, k_upSpeed);
      } else if (OI.getInstance().getElevatorDownButton()) {
        m_elevatorMotor.set(ControlMode.PercentOutput, k_downSpeed);
      } else {
        m_elevatorMotor.set(ControlMode.PercentOutput, k_holdSpeed);
      }
    }
  }
>>>>>>> Stashed changes
    /*
     * if (OI.getInstance().getElevatorUpButton() && m_state !=
     * ElevatorState.k_protectiveHold) { if
     * (Power.getInstance().getCurrent(OI.k_pdpElevatorMotor) > k_maxCurrent) { if
     * (!m_timerStarted) { m_holdTimer.start(); m_timerStarted = true; } if
     * (m_holdTimer.get() > k_maxStallTime) { m_holdTimer.stop();
     * m_holdTimer.reset(); m_timerStarted = false; m_state =
     * ElevatorState.k_protectiveHold; } else { m_state = ElevatorState.k_movingUp;
     * } } else { m_holdTimer.stop(); m_holdTimer.reset(); m_timerStarted = false;
     * m_state = ElevatorState.k_movingUp; } } else if
     * (OI.getInstance().getElevatorDownButton()) { m_state =
     * ElevatorState.k_movingDown; } else if (m_state !=
     * ElevatorState.k_protectiveHold) { m_state = ElevatorState.k_holding; }
     * 
     * double output = 0.0; if (m_state == ElevatorState.k_movingUp) { output =
     * k_upSpeed * k_upDirection; } else if (m_state == ElevatorState.k_movingDown)
     * { output = k_downSpeed * k_downDirection; } else { output = k_holdSpeed *
     * k_upDirection; } m_elevatorMotor.set(output);
     */
  }

  public void goToPosition(ElevatorPosition desiredPosition) {
<<<<<<< Updated upstream
    if (m_calibrated) {
      m_elevatorMotor.set(ControlMode.Position, desiredPosition.heightInEncoderTicks);
    }
=======

>>>>>>> Stashed changes
  }

  private void setManual(boolean manual) {
    m_manualControl = manual;
<<<<<<< Updated upstream
  }

  private void configureTalon() {
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_elevatorMotor.setSensorPhase(k_sensorPhase);
    m_elevatorMotor.configNominalOutputForward(0);
    m_elevatorMotor.configNominalOutputReverse(0);
    m_elevatorMotor.configPeakOutputForward(1);
    m_elevatorMotor.configPeakOutputReverse(-1);

    configureTalonPID(kP, kI, kD, kF);
  }

  private void configureTalonPID(double P, double I, double D, double F) {
    m_elevatorMotor.config_kP(0, P);
    m_elevatorMotor.config_kI(0, I);
    m_elevatorMotor.config_kD(0, D);
    m_elevatorMotor.config_kF(0, F);
  }

=======
  }

  private void configureTalon() {
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_elevatorMotor.setSensorPhase(k_sensorPhase);
    m_elevatorMotor.configNominalOutputForward(0);
    m_elevatorMotor.configNominalOutputReverse(0);
    m_elevatorMotor.configPeakOutputForward(1);
    m_elevatorMotor.configPeakOutputReverse(-1);

    configureTalonPID(kP, kI, kD, kF);
  }

  private void configureTalonPID(double P, double I, double D, double F) {
    m_elevatorMotor.config_kP(0, P);
    m_elevatorMotor.config_kI(0, I);
    m_elevatorMotor.config_kD(0, D);
    m_elevatorMotor.config_kF(0, F);
  }

>>>>>>> Stashed changes
  private class PIDUpdateListener implements Consumer<PIDList> {
    public void accept(PIDList values) {
      m_holding = true;
      configureTalonPID(values.P, values.I, values.D, values.F);
    }
  }

  private class CalibrationListener implements Consumer<EntryNotification> {
    public void accept(EntryNotification notification) {
      if (notification.value.getBoolean()) {
<<<<<<< Updated upstream
        m_elevatorMotor.setSelectedSensorPosition(0);
=======
>>>>>>> Stashed changes
        m_calibrated = true;
      } else {
        m_calibrated = false;
      }
    }
  }

  private class PositionListener implements Consumer<EntryNotification> { // This class will handle multiple entries
<<<<<<< Updated upstream
    private ElevatorPosition associatedPosition;
    private PositionListener(ElevatorPosition associatedPosition) {
      this.associatedPosition = associatedPosition;
    }
    public void accept(EntryNotification notification) {
      m_holding = false;
      setManual(false);
      goToPosition(associatedPosition);
    }
  }

  /* public static enum ElevatorState {
    k_autonomous, k_movingUp, k_movingDown, k_holding, k_protectiveHold;
  }
  */

  public enum ElevatorPosition {
    k_hatchRocketLow("Hatch Rocket Low", 0), k_hatchRocketMid("Hatch Rocket Mid", 2), k_hatchRocketHigh("Hatch Rocket High", 4), k_ballRocketLow("Ball Rocket Low", 6), k_ballRocketMid("Ball Rocket Mid", 8),
    k_ballRocketHigh("Ball Rocket High", 10), k_hatchCargo("Hatch Cargo", 12), k_ballCargo("Ball Cargo", 14);
    public final double heightInEncoderTicks;
    public final NetworkTableHandle handle = new NetworkTableHandle();
    public final WidgetProperties props;

    ElevatorPosition(String title, double heightInEncoderTicks) {
      this.heightInEncoderTicks = heightInEncoderTicks;
      props = new WidgetProperties(handle, title, BuiltInWidgets.kToggleButton, Elevator.getInstance().new PositionListener(this), false);
=======
    public void accept(EntryNotification notification) {
      NetworkTableEntry entry = notification.getEntry();
    }
  }

  private class PositionWidgetProperties extends WidgetProperties {
    public PositionWidgetProperties(NetworkTableHandle handle, String name) {
      super(handle, name, BuiltInWidgets.kToggleButton, positionListener, false);
>>>>>>> Stashed changes
    }

  }

  /* public static enum ElevatorState {
    k_autonomous, k_movingUp, k_movingDown, k_holding, k_protectiveHold;
  }
  */

  public enum ElevatorPosition {
    k_hatchRocketLow("Hatch Rocket Low", 0), k_hatchRocketMid("Hatch Rocket Mid", 2), k_hatchRocketHigh("Hatch Rocket High", 4), k_ballRocketLow("Ball Rocket Low", 6), k_ballRocketMid("Ball Rocket Mid", 8),
    k_ballRocketHigh("Ball Rocket High", 10), k_hatchCargo("Hatch Cargo", 12), k_ballCargo("Ball Cargo", 14);
    public final double heightInEncoderTicks;
    public final NetworkTableHandle handle = new NetworkTableHandle();
    public final WidgetProperties props;

    ElevatorPosition(String title, double heightInEncoderTicks) {
      this.heightInEncoderTicks = heightInEncoderTicks;
      props = new WidgetProperties(handle, title, BuiltInWidgets.kToggleButton, positionListener, false);
    }

  }
}