package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

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
     
     ShuffleboardTab driveTab = Shuffleboard.getTab(Keys.tab_Drive);
     ShuffleboardLayout motors = driveTab.getLayout(Keys.layout_Motors, BuiltInLayouts.kGrid)
         .withSize(3, 3).withPosition(0, 0).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));
     NetworkTableEntry ntLeftTalon = motors.add(Keys.leftTalon, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntLeftVictor = motors.add(Keys.leftVictor, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntRightTalon = motors.add(Keys.rightTalon, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntRightVictor = motors.add(Keys.rightVictor, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

     //TODO: make a better graph widget, with timestamps, and grab encoder values in a thread
     NetworkTableEntry ntEncoderVelocity = motors.add(Keys.driveEncoderVelocity, 0).withWidget(BuiltInWidgets.kGraph).getEntry();
     NetworkTableEntry ntEncoderPosition = motors.add(Keys.driveEncoderPosition, 0).withWidget(BuiltInWidgets.kGraph).getEntry();

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
    }
    private void setDriveMotors() {
        OI oi = OI.getInstance();
        m_DifferentialDrive.curvatureDrive(oi.getForwardSpeed(), oi.getCurvature(), false);
    }

    public void outputTelemetry() {
        ntLeftTalon.setDouble(leftTalon.getMotorOutputPercent());
        ntLeftVictor.setDouble(leftVictor.getMotorOutputPercent());
        ntRightTalon.setDouble(rightTalon.getMotorOutputPercent());
        ntRightVictor.setDouble(rightVictor.getMotorOutputPercent());
        ntEncoderVelocity.setDouble(leftTalon.getSelectedSensorVelocity());
        ntEncoderPosition.setDouble(leftTalon.getSelectedSensorPosition());
    }

    public void doRun() {
        setDriveMotors();
    }

    public void teleopInit() {
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
    }
}