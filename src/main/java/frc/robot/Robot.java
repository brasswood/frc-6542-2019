/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.vision.RedCamera;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX myTalon = new WPI_TalonSRX(0);
  RobotDrive myRobot;


  public Robot() {
    myRobot = new DifferentialDrive(0,1);
    myRobot.setExpiration (0.1);
  }

  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   // Create subsystem list
   private final SubsystemManager m_subsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(), Power.getInstance()));
  @Override
  public void robotInit() {
    myTalon.set(ControlMode.PercentOutput, 0);
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setVideoMode(RedCamera.kFormat, RedCamera.kWidth, RedCamera.kHeight, RedCamera.kFps);
    m_subsystemManager.initialize();
  }



  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    m_subsystemManager.run();
    m_subsystemManager.outputToSmartDashboard();
    myRobot.setSafetyEnabled(false);
    myRobot.drive(0.5, 0.5);
    Timer.delay(1.0);
    myRobot.drive(0.0, 0.0);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    m_subsystemManager.run();
    m_subsystemManager.outputToSmartDashboard();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
