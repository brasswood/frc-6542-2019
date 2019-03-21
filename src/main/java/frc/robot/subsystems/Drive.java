package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.io.File;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.OI;
import frc.robot.dashboard.PIDList;
import frc.robot.dashboard.PIDWidget;
import frc.robot.dashboard.WidgetProperties;
import frc.robot.dashboard.Keys;
import frc.robot.dashboard.LayoutBuilder;
import frc.robot.dashboard.NetworkTableHandle;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

// Wrapper class for DifferentialDrive

public class Drive extends Subsystem {

    private static Drive m_instance;
    public int max_vel = 0;
    private TalonSRX leftTalon = new TalonSRX(OI.k_canLeftDriveTalonID);
    private TalonSRX rightTalon = new TalonSRX(OI.k_canRightDriveTalonID);
    private VictorSPX leftVictor = new VictorSPX(OI.k_canLeftDriveVictorID);
    private VictorSPX rightVictor = new VictorSPX(OI.k_canRightDriveVictorID);
    private boolean m_autonEnabled;
    private Segment[] leftPath;
    private Segment[] rightPath;
    private int current_seg = 0;


    private final double k_wheelDiameter = 6;
    private final int k_encoderTicksPerRev = 4096;
    private final double k_inchesPerSecondToUnitsPer100Millis = k_encoderTicksPerRev / (k_wheelDiameter * Math.PI * 10);
    private final double k_maxVelInUnitsPer100Millis = 2953;
    private final double k_talonMaxOutput = 1023;
    private final double kP = 1.7, kI = 0, kD = 0, kF = k_talonMaxOutput / k_maxVelInUnitsPer100Millis;
    private double P = kP, I = kI, D = kD, F = kF;

    private final NetworkTableHandle ntLeftTalon = new NetworkTableHandle();
    private final NetworkTableHandle ntLeftVictor = new NetworkTableHandle();
    private final NetworkTableHandle ntRightTalon = new NetworkTableHandle();
    private final NetworkTableHandle ntRightVictor = new NetworkTableHandle();
    // TODO: make a better graph widget, with timestamps, and grab encoder values in
    // a thread
    private final NetworkTableHandle ntLeftEncoderVelocity = new NetworkTableHandle();
    private final NetworkTableHandle ntRightEncoderVelocity = new NetworkTableHandle();

    private final NetworkTableHandle ntLeftError = new NetworkTableHandle();
    private final NetworkTableHandle ntRightError = new NetworkTableHandle();

    private Drive() {
        ShuffleboardTab driveTab = Shuffleboard.getTab(Keys.Tabs.tab_Drive);
        new PIDWidget("Drive PID", driveTab, kP, kI, kD, kF).addListener(new PIDUpdateListener());
        WidgetProperties leftTalon = new WidgetProperties(ntLeftTalon, "Left Talon", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties rightTalon = new WidgetProperties(ntRightTalon, "Right Talon", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties rightVictor = new WidgetProperties(ntRightVictor, "Right Victor", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties leftVictor = new WidgetProperties(ntLeftVictor, "Left Victor", BuiltInWidgets.kNumberBar, null, 0);
        WidgetProperties leftEncoderVelocity = new WidgetProperties(ntLeftEncoderVelocity, "Left Encoder Velocity", BuiltInWidgets.kGraph, null, 0);
        WidgetProperties rightEncoderVelocity = new WidgetProperties(ntRightEncoderVelocity, "Right Encoder Velocity", BuiltInWidgets.kGraph, null, 0);
        WidgetProperties rightError = new WidgetProperties(ntLeftError, "Left Error", BuiltInWidgets.kGraph, null, 0);
        WidgetProperties leftError = new WidgetProperties(ntRightError, "Right Error", BuiltInWidgets.kGraph, null, 0);
        WidgetProperties[] widgets = {
            leftTalon, leftVictor, rightTalon, rightVictor, leftEncoderVelocity, rightEncoderVelocity, leftError, rightError
        };
        LayoutBuilder.buildLayout("Drive", BuiltInLayouts.kGrid, driveTab, widgets);
    }

    public static Drive getInstance() {
        if (m_instance == null) {
            m_instance = new Drive();
        }
        return m_instance;
    }

    public void init() {
        configureTalons();
    }

    public void outputTelemetry() {
        ntLeftTalon.setDouble(leftTalon.getMotorOutputPercent());
        ntLeftVictor.setDouble(leftVictor.getMotorOutputPercent());
        ntRightTalon.setDouble(rightTalon.getMotorOutputPercent());
        ntRightVictor.setDouble(rightVictor.getMotorOutputPercent());
        ntLeftEncoderVelocity.setDouble(leftTalon.getSelectedSensorVelocity());
        ntRightEncoderVelocity.setDouble(rightTalon.getSelectedSensorVelocity());
        
    }

    public void doRun() {
    }

    private void updatePID(double p, double i, double d, double f) {
        m_autonEnabled = false;
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        System.out.println("P: " + p + ", I: " + i + ", D: " + D + ", F: " + F);
        configureTalonPID();
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        double forward = OI.getInstance().getForwardSpeed();
        double curve = OI.getInstance().getCurvature();
        double lo = forward, ro = forward;
        if (curve < 0) {
            lo += (curve * forward);
        } else {
            ro -= (curve * forward);
        }
        leftTalon.set(ControlMode.PercentOutput, lo);
        rightTalon.set(ControlMode.PercentOutput, ro);
    }

    public void autonInit() {
        current_seg = 0;
        File leftPathFile = new File(Filesystem.getDeployDirectory(), "StraightLineTest.left.pf1.csv");
        File rightPathFile = new File(Filesystem.getDeployDirectory(), "StraightLineTest.right.pf1.csv");
        leftPath = PathfinderJNI.trajectoryDeserializeCSV(leftPathFile.toString());
        rightPath = PathfinderJNI.trajectoryDeserializeCSV(rightPathFile.toString());
        m_autonEnabled = true;
    }

    public void autonPeriodic() {
        if (m_autonEnabled) {
            double lo = 0, ro = 0;
            if (current_seg < leftPath.length) {
                ro = leftPath[current_seg].velocity * k_inchesPerSecondToUnitsPer100Millis;
                lo = rightPath[current_seg].velocity * k_inchesPerSecondToUnitsPer100Millis;
                current_seg++;
                leftTalon.set(ControlMode.Velocity, lo);
                rightTalon.set(ControlMode.Velocity, ro);
                System.out.println(lo + ", " + leftTalon.getClosedLoopError(0) + ", " + rightTalon.getClosedLoopError(0));
            } else if (current_seg == leftPath.length) {
                System.out.println("Path Complete");
                m_autonEnabled = false;
            } else {
                leftTalon.set(ControlMode.Velocity, 0);
                rightTalon.set(ControlMode.Velocity, 0);
            }
        } else {
            leftTalon.set(ControlMode.Velocity, 0);
            rightTalon.set(ControlMode.Velocity, 0);
        }
    }

    public void disabledPeriodic() {
    }

    private void configureTalons() {
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftTalon.setSensorPhase(OI.k_phaseSensor);
        rightTalon.setSensorPhase(OI.k_phaseSensor);
        leftTalon.configNominalOutputForward(0);
        rightTalon.configNominalOutputForward(0);
        leftTalon.configNominalOutputReverse(0);
        rightTalon.configNominalOutputReverse(0);
        leftTalon.configPeakOutputForward(1);
        rightTalon.configPeakOutputForward(1);
        leftTalon.configPeakOutputReverse(-1);
        rightTalon.configPeakOutputReverse(-1);

        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);

        rightTalon.setInverted(true);
        leftTalon.setInverted(false);
        rightVictor.setInverted(InvertType.FollowMaster);
        leftVictor.setInverted(InvertType.FollowMaster);

        configureTalonPID();
    }

    private void startMaxVelThread() {
        max_vel = 0;
        new Thread() { 
            public void run() { 
                while (true) { 
                    int vel = Math.abs(rightTalon.getSelectedSensorVelocity()); 
                    if (vel > max_vel) { 
                        max_vel = vel; 
                    }
                Timer.delay(0.1); 
                } 
            } 
        }.start();
    }

    private void configureTalonPID() {
        leftTalon.config_kP(0, P);
        leftTalon.config_kI(0, I);
        leftTalon.config_kD(0, D);
        leftTalon.config_kF(0, F);

        rightTalon.config_kP(0, P);
        rightTalon.config_kI(0, I);
        rightTalon.config_kD(0, D);
        rightTalon.config_kF(0, F);
    }

    private class PIDUpdateListener implements Consumer<PIDList> {
        public void accept(PIDList list) {
            updatePID(list.P, list.I, list.D, list.F);
        }
    }

}