package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

public class Elevator extends Subsystem {

    private Spark m_elevatorSpark = new Spark(0);

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
            m_elevatorSpark.set(0.5);
          } 
          else if (OI.getInstance().getElevatorDownButton() == true){
            m_elevatorSpark.set(-0.5);
          }
          else {
            m_elevatorSpark.set(0);
          }
    }

}