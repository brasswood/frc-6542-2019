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
    public int max_vel = 0;
     private WPI_TalonSRX leftTalon = new WPI_TalonSRX(OI.k_canLeftDriveTalonID);
     private WPI_TalonSRX rightTalon = new WPI_TalonSRX(OI.k_canRightDriveTalonID);
     private WPI_VictorSPX leftVictor = new WPI_VictorSPX(OI.k_canLeftDriveVictorID);
     private WPI_VictorSPX rightVictor = new WPI_VictorSPX(OI.k_canRightDriveVictorID);
     
     ShuffleboardTab driveTab = Shuffleboard.getTab(Keys.Tabs.tab_Drive);
     ShuffleboardLayout motors = driveTab.getLayout(Keys.Widgets.layout_Motors, BuiltInLayouts.kGrid)
         .withSize(3, 3).withPosition(0, 0).withProperties(Map.of("Number of columns", 1, "Number of rows", 1));
     NetworkTableEntry ntLeftTalon = motors.add(Keys.leftTalon, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntLeftVictor = motors.add(Keys.leftVictor, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntRightTalon = motors.add(Keys.rightTalon, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntRightVictor = motors.add(Keys.rightVictor, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

     //TODO: make a better graph widget, with timestamps, and grab encoder values in a thread
     NetworkTableEntry ntLeftEncoderVelocity = motors.add(Keys.leftDriveEncoderVelocity, 0).withWidget(BuiltInWidgets.kGraph).getEntry();
     NetworkTableEntry ntLeftEncoderPosition = motors.add(Keys.leftDriveEncoderPosition, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
     NetworkTableEntry ntRightEncoderVelocity = motors.add(Keys.rightDriveEncoderVelocity, 0).withWidget(BuiltInWidgets.kGraph).getEntry();
     NetworkTableEntry ntRightEncoderPosition = motors.add(Keys.rightDriveEncoderPosition, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

     NetworkTableEntry ntMaxVel = driveTab.add("Max Velocity", 0).getEntry();
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
        leftTalon.setSensorPhase(OI.k_phaseSensor);
    }

    private void setDriveMotors() {
        m_DifferentialDrive.curvatureDrive(OI.getInstance().getForwardSpeed(), OI.getInstance().getCurvature(), false);
    }

    public void outputTelemetry() {
        ntLeftTalon.setDouble(leftTalon.getMotorOutputPercent());
        ntLeftVictor.setDouble(leftVictor.getMotorOutputPercent());
        ntRightTalon.setDouble(rightTalon.getMotorOutputPercent());
        ntRightVictor.setDouble(rightVictor.getMotorOutputPercent());
        ntLeftEncoderVelocity.setDouble(leftTalon.getSelectedSensorVelocity());
        ntLeftEncoderPosition.setDouble(leftTalon.getSelectedSensorPosition());
        ntRightEncoderVelocity.setDouble(rightTalon.getSelectedSensorVelocity());
        ntRightEncoderPosition.setDouble(rightTalon.getSelectedSensorPosition());
        ntMaxVel.setDouble(max_vel);
    }

    public void doRun() {
        setDriveMotors();
    }

    public void teleopInit() {
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
        new Thread() {
            public void run() {
                int vel = leftTalon.getSelectedSensorVelocity();
                if (vel > max_vel) {
                    max_vel = vel;
                }
                System.out.println(vel);
                System.out.println(max_vel);
            }
        }.start();
        System.out.println("Teleopinit run!");

    }
}