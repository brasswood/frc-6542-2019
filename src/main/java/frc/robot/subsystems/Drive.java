package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

// Wrapper class for DifferentialDrive

public class Drive extends Subsystem {

    private static Drive m_instance;
    // TODO: Make these TalonSRX's
    private Spark m_LeftSpark = new Spark(OI.k_pwmLeftDrive1);
    private Spark m_RightSpark = new Spark(OI.k_pwmRightDrive1);
    private DifferentialDrive m_DifferentialDrive = new DifferentialDrive(m_LeftSpark, m_RightSpark);
    
    private Drive() {

    }

    public static Drive getInstance() {
        if (m_instance == null) {
            m_instance = new Drive();
        }
        return m_instance;
    }

    private void setDriveMotors() {
        OI oi = OI.getInstance();
        m_DifferentialDrive.curvatureDrive(oi.getForwardSpeed(), oi.getCurvature(), false);
    }

    public void outputTelemetry() {
        SmartDashboard.putData(Keys.leftSpark, m_LeftSpark);
        SmartDashboard.putData(Keys.rightSpark, m_RightSpark);
    }

    public void doRun() {
        setDriveMotors();
    }
}