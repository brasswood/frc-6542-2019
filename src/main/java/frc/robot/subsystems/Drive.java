package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

import java.io.File;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.OI;
import frc.robot.dashboard.Keys;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

// Wrapper class for DifferentialDrive

public class Drive extends Subsystem {

    private static Drive m_instance;
    public int max_vel = 0;
     private WPI_TalonSRX leftTalon = new WPI_TalonSRX(OI.k_canLeftDriveTalonID);
     private WPI_TalonSRX rightTalon = new WPI_TalonSRX(OI.k_canRightDriveTalonID);
     private WPI_VictorSPX leftVictor = new WPI_VictorSPX(OI.k_canLeftDriveVictorID);
     private WPI_VictorSPX rightVictor = new WPI_VictorSPX(OI.k_canRightDriveVictorID);
     private double kP, kI, kD, kF;
     private boolean pidUpdate = false;

     private Segment[] leftPath;
     private Segment[] rightPath;
     private int current_seg = 0;

     private final int k_wheelDiameter = 6;
     private final int k_encoderTicksPerRev = 4096;
     private final int k_encoderTicksPerInch = k_encoderTicksPerRev/k_wheelDiameter;
     
     ShuffleboardTab driveTab = Shuffleboard.getTab(Keys.Tabs.tab_Drive);
     ShuffleboardLayout motors = driveTab.getLayout(Keys.Widgets.layout_Motors, BuiltInLayouts.kGrid)
         .withSize(3, 3).withProperties(Map.of("Number of columns", 3, "Number of rows", 3));
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

     ShuffleboardLayout pid = driveTab.getLayout(Keys.Widgets.layout_PID, BuiltInLayouts.kList);
     NetworkTableEntry ntP = pid.add(Keys.driveP, kP).getEntry();
     NetworkTableEntry ntI = pid.add(Keys.driveI, kI).getEntry();
     NetworkTableEntry ntD = pid.add(Keys.driveD, kD).getEntry();
     NetworkTableEntry ntF = pid.add(Keys.driveF, kF).getEntry();
     NetworkTableEntry ntPIDToggle = pid.add(Keys.drivePIDToggle, false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

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
        configureTalons();
        m_DifferentialDrive.setRightSideInverted(false);
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
    }

    private void updatePID(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        configureTalonPID();
    }
    public void teleopInit() {
        max_vel = 0;
        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);
        new Thread() {
            public void run() {
                while (true) {
                    int vel = rightTalon.getSelectedSensorVelocity();
                    if (vel > max_vel) {
                        max_vel = vel;
                    }
                    Timer.delay(0.1);
                }
            }
        }.start();
        System.out.println("Teleopinit run!");
    }

    public void teleopPeriodic() {
        m_DifferentialDrive.curvatureDrive(OI.getInstance().getForwardSpeed(), OI.getInstance().getCurvature(), false);
    }

    public void autonInit() {
        current_seg = 0;
        File leftPathFile = new File(Filesystem.getDeployDirectory(), "StraightLineTest.left.pf1.csv");
        File rightPathFile = new File(Filesystem.getDeployDirectory(), "StraightLineTest.right.pf1.csv");
        leftPath = PathfinderJNI.trajectoryDeserializeCSV(leftPathFile.toString());
        rightPath = PathfinderJNI.trajectoryDeserializeCSV(rightPathFile.toString());
    }

    public void autonPeriodic() {
        double lo = 0, ro = 0;
        if (current_seg < leftPath.length) {
            lo = leftPath[current_seg].velocity *k_encoderTicksPerInch;
            ro = rightPath[current_seg].velocity *k_encoderTicksPerInch;
            current_seg++;
            leftTalon.set(ControlMode.Velocity, lo);
            rightTalon.set(ControlMode.Velocity, ro);
        } else {
            leftTalon.set(ControlMode.PercentOutput, 0);
            rightTalon.set(ControlMode.PercentOutput, 0);
            System.out.println("Path Complete");
        }

    }

    public void disabledPeriodic() {
        if (ntPIDToggle.getBoolean(false) != pidUpdate) {
            pidUpdate = !pidUpdate;
            updatePID(ntP.getDouble(kP), ntI.getDouble(kI), ntD.getDouble(kD), ntF.getDouble(kF));
        }
        // System.out.println(kP + ", " + kI + ", " + kD + ", " + kF);

    }

    private void configureTalons() {
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftTalon.setSensorPhase(OI.k_phaseSensor);
        leftTalon.configNominalOutputForward(0);
        rightTalon.configNominalOutputForward(0);
        leftTalon.configNominalOutputReverse(0);
        rightTalon.configNominalOutputReverse(0);
        leftTalon.configPeakOutputForward(1);
        rightTalon.configPeakOutputForward(1);
        leftTalon.configPeakOutputReverse(-1);
        rightTalon.configPeakOutputReverse(-1);

        rightTalon.setInverted(true);
        leftTalon.setInverted(false);
        rightVictor.setInverted(InvertType.FollowMaster);
        leftVictor.setInverted(InvertType.FollowMaster);

        configureTalonPID();
    }

    private void configureTalonPID() {
        leftTalon.config_kP(0, kP);
        leftTalon.config_kI(0, kI);
        leftTalon.config_kD(0, kD);
        leftTalon.config_kF(0, kF);

        rightTalon.config_kP(0, kP);
        rightTalon.config_kI(0, kI);
        rightTalon.config_kD(0, kD);
        rightTalon.config_kF(0, kF);
    }
}