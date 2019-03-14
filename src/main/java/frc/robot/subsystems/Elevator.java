package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Elevator extends Subsystem {

    private Spark m_elevatorSpark = new Spark(OI.k_pwmElevatorMotor);
    private static Elevator m_instance;

    private final double k_upDirection = 1.0;
    private final double k_downDirection = -1.0;
    private final double k_upSpeed = 1;
    private final double k_downSpeed = 0.03;
    private final double k_holdSpeed = 0.21;
    private final double k_topPosition = 0;

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
        if (OI.getInstance().getElevatorUpButton() == true){
            double output = 0;
            m_elevatorSpark.set(k_upSpeed*k_upDirection);
        } 
        else if (OI.getInstance().getElevatorDownButton() == true){
            m_elevatorSpark.set(k_downSpeed*k_downDirection);
        }
        else {
            m_elevatorSpark.set(k_holdSpeed*k_upDirection);
        }
    }

}