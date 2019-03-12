package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Elevator extends Subsystem {

    private Spark m_elevatorSpark = new Spark(OI.k_pwmElevatorMotor);

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;
    private double kHoldSetpoint = 0;
    private double kPercentTolerance = 5;

    private static Elevator m_instance;

    private ShuffleboardLayout elevatorTuningLayout = Shuffleboard.getTab(Keys.Tabs.subsystems).getLayout(Keys.Layouts.elevatorPIDTune, BuiltInLayouts.kList);
    private NetworkTableEntry ntP = elevatorTuningLayout.add(Keys.elevatorP, kP).getEntry();
    private NetworkTableEntry ntI = elevatorTuningLayout.add(Keys.elevatorI, kI).getEntry();
    private NetworkTableEntry ntD = elevatorTuningLayout.add(Keys.elevatorD, kD).getEntry();
    private NetworkTableEntry ntF = elevatorTuningLayout.add(Keys.elevatorF, kF).getEntry();
    private NetworkTableEntry ntHoldSetpoint = elevatorTuningLayout.add(Keys.elevatorHoldSetpoint, kHoldSetpoint).getEntry();
    private NetworkTableEntry ntHoldPercentTolerance = elevatorTuningLayout.add(Keys.elevatorPercentTolerance, kPercentTolerance).getEntry();
    private PIDController m_elevatorHolder = new PIDController(kP, kI, kD, kF, Power.getInstance().pidChan, m_elevatorSpark);

    private Elevator() {}

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
        
    }

    @Override
    public void doRun() {
        if ((ntP.getDouble(kP) != kP) || (ntI.getDouble(kI) != kI) || (ntD.getDouble(kD) != kD) || (ntF.getDouble(kF) != kF)) {
          kP = ntP.getDouble(kP);
          kI = ntI.getDouble(kI);
          kD = ntD.getDouble(kD);
          kF = ntF.getDouble(kF);
          m_elevatorHolder.setPID(kP, kI, kD, kF);
        }
        if (OI.getInstance().getElevatorUpButton() == true){
          if (m_elevatorHolder.isEnabled()) {m_elevatorHolder.disable();}
          m_elevatorSpark.set(1.0);
        } 
        else if (OI.getInstance().getElevatorDownButton() == true){
          if (m_elevatorHolder.isEnabled()) {m_elevatorHolder.disable();}
          m_elevatorSpark.set(-0.5);
        }
        else {
          if (m_elevatorHolder.getSetpoint() != kHoldSetpoint) {
            m_elevatorHolder.setSetpoint(kHoldSetpoint);
          }
          m_elevatorHolder.setPercentTolerance(kPercentTolerance);
          if (!m_elevatorHolder.isEnabled()) {
            m_elevatorHolder.enable();
          }
        }
    }

}