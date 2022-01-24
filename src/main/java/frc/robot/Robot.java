// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    System.out.println("Robot Init");
  }

  @Override
  public void robotPeriodic() {
    System.out.println("Robot Periodic");
  }

  @Override
  public void autonomousInit() {
    System.out.println("Auto Init");
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("Auto Periodic");
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop Init");
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("Teleop Periodic");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
