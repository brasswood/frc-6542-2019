package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Elevator extends Subsystem {

    private WPI_TalonSRX m_elevatorMotor = new WPI_TalonSRX(OI.k_canElevatorMotor);
    private static Elevator m_instance;

    private Elevator() {}

    public static Elevator getInstance() {
        if (m_instance == null) {
            m_instance = new Elevator();
        }
        return m_instance;
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(Keys.elevatorSpark, m_elevatorMotor);
    }

    @Override
    public void init() {
        
    }

    @Override
    public void doRun() {
        if (OI.getInstance().getElevatorUpButton() == true){
            m_elevatorMotor.set(1.0);
          } 
          else if (OI.getInstance().getElevatorDownButton() == true){
            m_elevatorMotor.set(-1.0);
          }
          else {
            m_elevatorMotor.set(0);
          }
    }

}