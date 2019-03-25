package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {

    private static OI m_instance;
    private XboxController m_controlPad = new XboxController(0); /* Must construct specific controller (ie.
    * XboxController(), Joystick()) */
    private Joystick m_controlStick = new Joystick(1);

    // Declare PWM and CAN ports
    public static final int k_canElevatorMotor = 5;
    public static final int k_canLeftDriveTalonID = 1;
    public static final int k_canLeftDriveVictorID = 3;
    public static final int k_canRightDriveTalonID = 2;
    public static final int k_canRightDriveVictorID = 4;
    public static final int k_canPDPID = 0;
    public static final int k_pwmIntakeMotor = 0;

    // Declare PDP ports
    public static final int k_pdpLeftDrive1 = 1;
    public static final int k_pdpLeftDrive2 = -1;
    public static final int k_pdpRightDrive1 = 0;
    public static final int k_pdpRightDrive2 = -1;
    public static final int k_pdpElevatorMotor = 12;

    //Our encoder is backwards. Fix that.
    public static final boolean k_phaseSensor = true;

    // Joystick (or XboxController or whatever we use) mappings
    private static final int k_rightThrottleAxis = 3;
    private static final int k_leftThrottleAxis = 2;
    private static final int k_povUp = 0;
    private static final int k_povRight = 90;
    private static final int k_povDown = 180;
    private static final int k_povLeft = 270;

    public int m_pov = 0;
    public int m_povPrev;


    public void init() {
    }

    public void update() {
        m_povPrev = m_pov;
        m_pov = m_controlStick.getPOV();
    }

    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        } else {
            return ((1/(1-deadband))*(input-(Math.signum(input)*deadband)));
        }
    }

    public double getForwardSpeed() {
        return m_controlPad.getRawAxis(k_rightThrottleAxis) - m_controlPad.getRawAxis(k_leftThrottleAxis);
    }

    public double getCurvature() {
        return m_controlPad.getX(Hand.kLeft);
    }

    public double getElevatorManualSpeed() {
        return deadband(m_controlStick.getX(), 0.1);
    }

    public boolean getElevatorUpButton() {
        return ((m_pov == k_povRight) && (m_povPrev != k_povRight));
    }

    public boolean getElevatorDownButton(){
        return ((m_pov == k_povLeft) && (m_povPrev != k_povLeft));
    }

    public double getIntakeManualSpeed() {
        return deadband(m_controlStick.getY(), 0.1);
    }

    public boolean getIntakeUpButton() {
        return ((m_pov == k_povDown) && (m_povPrev != k_povDown));
    }

    public boolean getIntakeDownButton() {
        return ((m_pov == k_povUp) && (m_povPrev != k_povUp));
    }

    public static OI getInstance() {
        if (m_instance == null) {
            m_instance = new OI();
        }
        return m_instance;
    }
}