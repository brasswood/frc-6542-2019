package frc.robot.subsystems;

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

     private SimpleWidget widget_leftTalon, widget_leftVictor, widget_rightTalon, widget_rightVictor;

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
        ShuffleboardTab driveTab = Shuffleboard.getTab(Keys.tab_Drive);
        ShuffleboardLayout motors = driveTab.getLayout(Keys.layout_Motors, BuiltInLayouts.kGrid)
            .withSize(16, 16).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
        widget_leftTalon = motors.add(Keys.leftTalon, 0).withWidget(BuiltInWidgets.kSpeedController);
        widget_leftVictor = motors.add(Keys.leftVictor, 0).withWidget(BuiltInWidgets.kSpeedController);
        widget_rightTalon = motors.add(Keys.rightTalon, 0).withWidget(BuiltInWidgets.kSpeedController);
        widget_rightVictor = motors.add(Keys.rightVictor, 0).withWidget(BuiltInWidgets.kSpeedController);
    }
    private void setDriveMotors() {
        OI oi = OI.getInstance();
        m_DifferentialDrive.curvatureDrive(oi.getForwardSpeed(), oi.getCurvature(), false);
    }

    public void outputTelemetry() {
        widget_leftTalon.getEntry().setDouble(leftTalon.get());
        widget_leftVictor.getEntry().setDouble(leftVictor.get());
        widget_rightTalon.getEntry().setDouble(rightTalon.get());
        widget_rightVictor.getEntry().setDouble(rightVictor.get());

    }

    public void doRun() {
        setDriveMotors();
    }

    public void teleopInit() {
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
                // maybe bad coding practice but I want to practice using anonymous classes
        new Thread(new Runnable() {
            public void run() {
                while (true) {
                    try {
                        Thread.sleep(1);
                    } catch (Exception ex) {
                        // Do Nothing
                    }
                    /* Grab the latest signal update from our ~1ms frame update */
                    double velocity = leftTalon.getSelectedSensorVelocity(0);
                    SmartDashboard.putNumber(Keys.driveEncoderVelocity, velocity);
                }
            }
        }).start();
    }
}