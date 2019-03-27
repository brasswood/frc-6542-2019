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
  private boolean m_timerStarted = false;
  private boolean m_calibrated;
  private boolean m_manualControl = true;
  private ElevatorPosition m_elevatorPosition = ElevatorPosition.k_bottom;

  private boolean m_holding = true;

  private final double k_upSpeed = 1;
  private final double k_downSpeed = -0.5;
  private final double k_holdSpeed = 0.1;
  private final double k_maxCurrent = 70;
  private final double k_maxStallTime = 2.5;

  private final boolean k_sensorPhase = false;

  private final NetworkTableHandle ntOutput = new NetworkTableHandle();
  private final NetworkTableHandle ntPosition = new NetworkTableHandle();
  private final NetworkTableHandle ntSetpoint = new NetworkTableHandle();
  private final NetworkTableHandle ntError = new NetworkTableHandle();
  private final NetworkTableHandle ntCalibrated = new NetworkTableHandle();
  private final NetworkTableHandle ntManual = new NetworkTableHandle();
  private final NetworkTableHandle ntHolding = new NetworkTableHandle();

  private final double kP = 0, kI = 0, kD = 0, kF = 0;
  private final int k_error = 200;


  private Elevator() {
    new PIDWidget("Elevator PID", Shuffleboard.getTab(Keys.Tabs.tab_Calibrate), kP, kI, kD, kF).addListener(new PIDUpdateListener());
    WidgetProperties output = new WidgetProperties(ntOutput, "Output", BuiltInWidgets.kNumberBar, null, 0);
    WidgetProperties position = new WidgetProperties(ntPosition, "Position", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties setpoint = new WidgetProperties(ntSetpoint, "Setpoint", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties manual = new WidgetProperties(ntManual, "Manual", BuiltInWidgets.kBooleanBox, null, false);
    WidgetProperties error = new WidgetProperties(ntError, "Error", BuiltInWidgets.kTextView, null, 0);
    WidgetProperties holding = new WidgetProperties(ntHolding, "Holding", BuiltInWidgets.kBooleanBox, null, false);
    WidgetProperties calibrated = new WidgetProperties(ntCalibrated, "CALIBRATED", BuiltInWidgets.kBooleanBox, null, false);
    WidgetProperties calibrateButton = new WidgetProperties(null, "CALIBRATE", BuiltInWidgets.kToggleButton, new CalibrationListener(), false);
    
    ArrayList<WidgetProperties> positionsArray = new ArrayList<WidgetProperties>();
    for (ElevatorPosition p : ElevatorPosition.values()) {
      positionsArray.add(new WidgetProperties(p.handle, p.title, BuiltInWidgets.kToggleButton, new PositionListener(p), false));
    }
    LayoutBuilder.buildLayout("Elevator Positions", BuiltInLayouts.kList, Shuffleboard.getTab(Keys.Tabs.tab_Control), 2, 5, positionsArray.toArray(new WidgetProperties[positionsArray.size()]));
    
    WidgetProperties[] elevatorWidgetArray = {manual, holding, output, position, setpoint, error};
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
    ntManual.setBoolean(m_manualControl);
    ntHolding.setBoolean(m_holding);
    if (!m_manualControl) {
      ntSetpoint.setDouble(m_elevatorMotor.getClosedLoopTarget());
      ntError.setDouble(m_elevatorMotor.getClosedLoopError());
    }
  }

  @Override
  public void init() {
    configureTalon();
  }

  @Override
  public void doRun() {
    // Poll controls to set manual mode
    if (OI.getInstance().getElevatorManualSpeed() != 0) {
      m_elevatorMotor.set(ControlMode.PercentOutput, OI.getInstance().getElevatorManualSpeed());
      m_manualControl = true;
    } else if (OI.getInstance().getElevatorDownButton()) {
      requestPosition(m_elevatorPosition.previous());
      m_manualControl = false;
    } else if (OI.getInstance().getElevatorUpButton()) {
      m_manualControl = false;
      requestPosition(m_elevatorPosition.next());
    } else if (m_manualControl) {
      m_elevatorMotor.set(ControlMode.PercentOutput, k_holdSpeed);
    }
  }

  public void requestPosition(ElevatorPosition desiredPosition) {
    if (m_calibrated) {
      m_elevatorPosition = desiredPosition;
      m_elevatorMotor.set(ControlMode.Position, desiredPosition.heightInEncoderTicks);
      ntSetpoint.setDouble(desiredPosition.heightInEncoderTicks);
    }
  }

  private void configureTalon() {
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_elevatorMotor.setSensorPhase(k_sensorPhase);
    m_elevatorMotor.configNominalOutputForward(0);
    m_elevatorMotor.configNominalOutputReverse(0);
    m_elevatorMotor.configPeakOutputForward(1);
    m_elevatorMotor.configPeakOutputReverse(-1);
    m_elevatorMotor.configAllowableClosedloopError(0, k_error);

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
      m_manualControl = false;
      requestPosition(listenFor);
    }
  }

  public enum ElevatorPosition {
    k_bottom("Bottom", 0, 0), k_hatchRocketLow("Hatch Rocket Low", 20480, 1), k_hatchRocketMid("Hatch Rocket Mid", 2, 2), k_hatchRocketHigh("Hatch Rocket High", 4, 3), k_ballRocketLow("Ball Rocket Low", 6, 4), k_ballRocketMid("Ball Rocket Mid", 8, 5),
    k_ballRocketHigh("Ball Rocket High", 10, 6), k_hatchCargo("Hatch Cargo", 12, 7), k_ballCargo("Ball Cargo", 14, 8);
    public final double heightInEncoderTicks;
    public final String title;
    private int index;
    private static ElevatorPosition[] orderedValues = {k_bottom, k_hatchRocketLow, k_hatchRocketMid, k_hatchRocketHigh, k_ballRocketLow, k_ballRocketMid, k_ballRocketHigh, k_hatchCargo, k_ballCargo};
    final NetworkTableHandle handle;

    ElevatorPosition(String title, double heightInEncoderTicks, int index) {
      this.heightInEncoderTicks = heightInEncoderTicks;
      this.title = title;
      this.index = index;
      this.handle = new NetworkTableHandle();
      
    }

    public ElevatorPosition next() {
      if (orderedValues.length - 1 > index) {
        return orderedValues[index+1];
      } else {
        return this;
      }
    }

    public ElevatorPosition previous() {
      if (index > 0) {
        return orderedValues[index-1];
      } else {
        return this;
      }
    }
  }
}