package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
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

  private boolean m_holding = true;

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

  private final double kP = 0, kI = 0, kD = 0, kF = 0;


  private Elevator() {
    new PIDWidget("Elevator PID", Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), kP, kI, kD, kF).addListener(new PIDUpdateListener());
    WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
    WidgetProperties position = new WidgetProperties(ntPosition, "Position", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties error = new WidgetProperties(ntError, "Error", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties calibrated = new WidgetProperties(ntCalibrated, "CALIBRATED", BuiltInWidgets.kBooleanBox, null, false);
    WidgetProperties calibrateButton = new WidgetProperties(null, "CALIBRATE", BuiltInWidgets.kToggleButton, new CalibrationListener(), false);
    
    ArrayList<WidgetProperties> positionsArray = new ArrayList<WidgetProperties>();
    for (ElevatorPosition p : ElevatorPosition.values()) {
      positionsArray.add(new WidgetProperties(p.handle, p.title, BuiltInWidgets.kToggleButton, new PositionListener(p), false));
    }
    LayoutBuilder.buildLayout("Elevator Positions", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Control), 2, 5, positionsArray.toArray(new WidgetProperties[positionsArray.size()]));
    
    WidgetProperties[] elevatorWidgetArray = {output, position, setpoint, error};
    LayoutBuilder.buildLayout("Elevator", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Debug), 5, 5, elevatorWidgetArray);
    LayoutBuilder.buildLayout("Elevator", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), 1, 1, new WidgetProperties[]{calibrated, calibrateButton});

  }

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }

  @Override
  public void outputTelemetry() {
    ntOutput.setDouble(m_elevatorMotor.getMotorOutputPercent());
    ntPosition.setDouble(m_elevatorMotor.getSelectedSensorPosition());
    if (!m_manualControl) {
      ntSetpoint.setDouble(m_elevatorMotor.getClosedLoopTarget());
      ntError.setDouble(m_elevatorMotor.getClosedLoopError());
    }
  }

  @Override
  public void init() {
    m_currentHeight = 0;
    configureTalon();
  }

  @Override
  public void doRun() {
    // Poll controls to set manual mode
    if (OI.getInstance().getElevatorUpButton() || OI.getInstance().getElevatorDownButton()) {
      setManual(true);
    }
    if (m_manualControl) {
      if (OI.getInstance().getElevatorUpButton()) {
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
    if (m_calibrated) {
      m_elevatorMotor.set(ControlMode.Position, desiredPosition.heightInEncoderTicks);
      ntSetpoint.setDouble(desiredPosition.heightInEncoderTicks);
    }
  }

  private void setManual(boolean manual) {
    m_manualControl = manual;
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

  private class PIDUpdateListener implements Consumer<PIDList> {
    public void accept(PIDList values) {
      m_holding = true;
      configureTalonPID(values.P, values.I, values.D, values.F);
    }
  }

  private class CalibrationListener implements Consumer<EntryNotification> {
    public void accept(EntryNotification notification) {
      if (notification.value.getBoolean()) {
        m_elevatorMotor.setSelectedSensorPosition(0);
        m_calibrated = true;
        ntCalibrated.setBoolean(true);
      } else {
        m_calibrated = false;
        ntCalibrated.setBoolean(false);
      }
    }
  }

  public class PositionListener implements Consumer<EntryNotification> { // This class will handle multiple entries
    private ElevatorPosition listenFor;
    private PositionListener(ElevatorPosition listenFor) {
      this.listenFor = listenFor;
    }
    public void accept(EntryNotification notification) {
      m_holding = false;
      setManual(false);
      goToPosition(listenFor);
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
    public final String title;
    final NetworkTableHandle handle;

    ElevatorPosition(String title, double heightInEncoderTicks) {
      this.heightInEncoderTicks = heightInEncoderTicks;
      this.title = title;
      this.handle = new NetworkTableHandle();
    }

  }

  /* public static enum ElevatorState {
    k_autonomous, k_movingUp, k_movingDown, k_holding, k_protectiveHold;
  }
  */

}