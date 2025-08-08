// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.mw_lib.proxy_server.ProxyServer;

public class Robot extends TimedRobot {
  private RobotContainer robot_container_;

  @Override
  public void robotInit() {
    robot_container_ = RobotContainer.getInstance();
    OI.configureBindings();
    ProxyServer.configureServer();
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    robot_container_.outputTelemetry();

    // updates data from chassis proxy server
    ProxyServer.updateData();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    ProxyServer.syncMatchData();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
