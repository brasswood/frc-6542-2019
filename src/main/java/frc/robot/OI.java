package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {

    private static OI m_instance;
    private XboxController m_controlPad = new XboxController(0); /* Must construct specific controller (ie.
    * XboxController(), Joystick()) */
    private int m_elevatorPos = 0;
    private int m_povPrev = 0;

    // Declare PWM and CAN ports
    public static final int k_pwmElevatorMotor = 0;
    public static final int k_pwmIntakeMotor = 1;
    public static final int k_canLeftDriveTalonID = 1;
    public static final int k_canLeftDriveVictorID = 3;
    public static final int k_canRightDriveTalonID = 2;
    public static final int k_canRightDriveVictorID = 4;
    public static final int k_canPDPID = 0;

    // Declare PDP ports
    public static final int k_pdpLeftDrive1 = 1;
    public static final int k_pdpLeftDrive2 = -1;
    public static final int k_pdpRightDrive1 = 0;
    public static final int k_pdpRightDrive2 = -1;
    public static final int k_pdpElevatorMotor = 12;

    //Our encoder is backwards. Fix that.
    public static final boolean k_phaseSensor = true;

    // Maximum elevator index
    private static final int k_maxElevatorPos = 3;

    // Joystick (or XboxController or whatever we use) mappings
    private static final int k_rightThrottleAxis = 3;
    private static final int k_leftThrottleAxis = 2;
    private static final int k_povUp = 90;
    private static final int k_povDown = 270;
    private static final int k_leftButton = 4;
    private static final int k_rightButton = 5;

    public void init() {
    }

    public void update() {
        /*int pov = m_controlPad.getPOV();
        if (m_povPrev != pov) {
            if (pov == k_povUp) {
                m_elevatorPos++;
            } else if (pov == k_povDown) {
                m_elevatorPos--;
            }
            if (m_elevatorPos > k_maxElevatorPos) {
                m_elevatorPos = k_maxElevatorPos;
            } else if (m_elevatorPos < 0) {
                m_elevatorPos = 0;
            }
        }
        m_povPrev = m_controlPad.getPOV();
        */
    }

    public double getForwardSpeed() {
        return m_controlPad.getRawAxis(k_rightThrottleAxis) - m_controlPad.getRawAxis(k_leftThrottleAxis);
    }

    public double getCurvature() {
        return m_controlPad.getX(Hand.kLeft);
    }

    public int getElevatorPos() {
        return m_elevatorPos;
    }

    public boolean getElevatorUpButton() {
        return m_controlPad.getBumper(Hand.kRight);
    }

    public boolean getElevatorDownButton(){
        return m_controlPad.getBumper(Hand.kLeft);
    }

    public int getIntakeButton() {
        if (m_controlPad.getAButton()) {
            return -1;
        } else if (m_controlPad.getBButton()) {
            return 1;
        } else {
            return 0;
        }
    }

    public static OI getInstance() {
        if (m_instance == null) {
            m_instance = new OI();
        }
        return m_instance;
    }
}