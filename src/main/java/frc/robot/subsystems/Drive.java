package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.OI;
import frc.robot.dashboard.Keys;

// Wrapper class for DifferentialDrive

public class Drive extends Subsystem {

    private static Drive m_instance;
    // TODO: Make these TalonSRX's
     private WPI_TalonSRX leftTalon = new WPI_TalonSRX(OI.k_canLeftDriveTalonID);
     private WPI_TalonSRX rightTalon = new WPI_TalonSRX(OI.k_canRightDriveTalonID);
     private WPI_VictorSPX leftVictor = new WPI_VictorSPX(OI.k_canLeftDriveVictorID);
     private WPI_VictorSPX rightVictor = new WPI_VictorSPX(OI.k_canRightDriveVictorID);

     private DifferentialDrive m_DifferentialDrive = new DifferentialDrive(leftTalon, rightTalon);
    
    private Drive() {

    }

    public static Drive getInstance() {
        if (m_instance == null) {
            m_instance = new Drive();
        }
        return m_instance;
    }

    public void init() {
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
    }
    private void setDriveMotors() {
        OI oi = OI.getInstance();
        m_DifferentialDrive.curvatureDrive(oi.getForwardSpeed(), oi.getCurvature(), false);
    }

    public void outputTelemetry() {
        SmartDashboard.putData(Keys.leftTalon, leftTalon);
        SmartDashboard.putData(Keys.rightTalon, rightTalon);
        SmartDashboard.putData(Keys.leftVictor, leftVictor);
        SmartDashboard.putData(Keys.rightVictor, rightVictor);
    }

    public void doRun() {
        setDriveMotors();
    }
}