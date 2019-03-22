/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.dashboard.Keys;
import frc.robot.subsystems.*;
import frc.robot.vision.RedCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   // Create subsystem list
   private final SubsystemManager m_subsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(), Elevator.getInstance(), Power.getInstance(), Intake.getInstance()));
  @Override
  public void robotInit() {
    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setVideoMode(RedCamera.kFormat, RedCamera.kWidth, RedCamera.kHeight, RedCamera.kFps);
    Shuffleboard.getTab(Keys.Tabs.tab_Control).add(cam).withSize(2, 2).withPosition(0, 0);
    m_subsystemManager.initialize();
  }



  @Override
  public void autonomousInit() {
    Drive.getInstance().autonInit();
  }

  @Override
  public void autonomousPeriodic() {
    m_subsystemManager.run();
    m_subsystemManager.outputToSmartDashboard();
    Drive.getInstance().autonPeriodic();
  }

  @Override
  public void teleopInit() {
    Drive.getInstance().teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    m_subsystemManager.run();
    m_subsystemManager.outputToSmartDashboard();
    Drive.getInstance().teleopPeriodic();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    Drive.getInstance().disabledPeriodic();
  }

}
