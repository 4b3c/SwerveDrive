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
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  double x;
  double y;
  double twist;
  double joystickAngle;
  double joystickMag;

  @Override
  public void robotInit() {
    Map.initialAngle = Map.gyro.getYaw();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    Map.elapsedTime = 0;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Map.elapsedTime = 0;
  }

  @Override
  public void teleopPeriodic() {
    x = Map.driver.getRawAxis(0) / 2;
    y = Map.driver.getRawAxis(1) / 2;
    twist = Map.driver.getRawAxis(4) / 2;
    joystickAngle = 180 + (Math.atan2(y, -x) / (Math.PI) * 180);
    joystickMag = Math.sqrt(x * x + y * y);

    Map.swerve.swerveDrive(Math.round(joystickAngle / 45) * 45, joystickMag, twist);
    Map.swerve.odometry();

    if (Map.driver.getRawButton(6)) {
      Map.initialAngle = Map.gyro.getYaw();
    }
    if (Map.driver.getRawButton(3)) {
      Map.swerve.xPos = 0;
      Map.swerve.yPos = 0;
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
