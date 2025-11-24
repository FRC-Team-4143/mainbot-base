// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainConstants.DrivetrainStates;

import java.util.Optional;

public class Robot extends TimedRobot {

  private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective
  private RobotContainer robot_container_;

  public Robot() {
    // Load the subsystems
    robot_container_ = RobotContainer.getInstance();

    // Configure External Interfaces
    OI.configureBindings();
    ProxyServer.configureServer();
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    // updates data from chassis proxy server
    ProxyServer.updateData();

    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // run the main robot loop for each subsystem
    robot_container_.doControlLoop();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() != alliance_) {
        alliance_ = alliance.get();
      }
    }
  }

  @Override
  public void autonomousInit() {
    DrivetrainSubsystem.getInstance().zeroCurrentPosition();
  }

  @Override
  public void autonomousPeriodic() {
    if (DrivetrainSubsystem.getInstance().getCurrentPosition().getX() < 9.9) {
      DrivetrainSubsystem.getInstance().setAutoCommand(0.25, 0);
    } else if (DrivetrainSubsystem.getInstance().getCurrentPosition().getX() > 10.1) {
      DrivetrainSubsystem.getInstance().setAutoCommand(-0.25, 0);
    } else {
      DrivetrainSubsystem.getInstance().setAutoCommand(0, 0);
    }
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    ProxyServer.syncMatchData();
    CommandScheduler.getInstance().cancelAll();
    DrivetrainSubsystem.getInstance().setWantedState(DrivetrainStates.TELE_OP);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
