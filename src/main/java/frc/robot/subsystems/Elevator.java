package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Elevator extends Subsystem {

    private Spark m_elevatorSpark = new Spark(OI.k_pwmElevatorMotor);
    private static Elevator m_instance;
    private Timer m_holdTimer = new Timer();
    private boolean m_timerStarted = false;
    private ElevatorState m_state = ElevatorState.k_holding;

    private final double k_upDirection = 1.0;
    private final double k_downDirection = -1.0;
    private final double k_upSpeed = 1;
    private final double k_downSpeed = 0.03;
    private final double k_holdSpeed = 0.21;
    private final double k_topPosition = 0;
    private final double k_maxCurrent = 70;
    private final double k_maxStallTime = 2.5;

    private Elevator() {}

    public static Elevator getInstance() {
        if (m_instance == null) {
            m_instance = new Elevator();
        }
        return m_instance;
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(Keys.elevatorSpark, m_elevatorSpark);
    }

    @Override
    public void init() {
        
    }

    @Override
    public void doRun() {
      if (OI.getInstance().getElevatorUpButton() && m_state != ElevatorState.k_protectiveHold) {
        if (Power.getInstance().getCurrent(OI.k_pdpElevatorMotor) > k_maxCurrent) {
          if (!m_timerStarted) {
            m_holdTimer.start();
            m_timerStarted = true;
          }
          if (m_holdTimer.get() > k_maxStallTime) {
            m_holdTimer.stop();
            m_holdTimer.reset();
            m_timerStarted = false;
            m_state = ElevatorState.k_protectiveHold;
          } else {
            m_state = ElevatorState.k_movingUp;
          }
        } else {
          m_holdTimer.stop();
          m_holdTimer.reset();
          m_timerStarted = false;
          m_state = ElevatorState.k_movingUp;
        }
      } else if (OI.getInstance().getElevatorDownButton()) {
        m_state = ElevatorState.k_movingDown;
      } else if (m_state != ElevatorState.k_protectiveHold) {
          m_state = ElevatorState.k_holding;
      }

    double output = 0.0;
    if (m_state == ElevatorState.k_movingUp) {
      output = k_upSpeed * k_upDirection;
    } else if (m_state == ElevatorState.k_movingDown) {
      output = k_downSpeed * k_downDirection;
    } else {
      output = k_holdSpeed * k_upDirection;
    }
      m_elevatorSpark.set(output);
    }
    
    public static enum ElevatorState {
      k_movingUp, k_movingDown, k_holding, k_protectiveHold;
    }
}