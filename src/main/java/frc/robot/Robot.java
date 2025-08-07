// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.mw_lib.auto.Auto;
import frc.mw_lib.auto.AutoManager;
import frc.mw_lib.logging.Elastic;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;

public class Robot extends TimedRobot {
  private RobotContainer robot_container_;
  private Alliance alliance_ = Alliance.Blue;

  @Override
  public void robotInit() {
    robot_container_ = RobotContainer.getInstance();
    OI.configureBindings();
    ProxyServer.configureServer();

    SmartDashboard.putData(
        "Snapshot", Commands.runOnce(() -> ProxyServer.snapshot("Test Snapshot")));
    SmartDashboard.putData("Sync Match Data", Commands.runOnce(() -> ProxyServer.syncMatchData()));
  }

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // tell the subsystems to output telemetry to smartdashboard
    robot_container_.outputTelemetry();

    // updates data from chassis proxy server
    ProxyServer.updateData();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putString("Intake Preference", OI.intake_preference.toString());
  }

  @Override
  public void disabledInit() {
    SwerveDrivetrain.getInstance().setDriveMode(DriveMode.IDLE);
  }

  @Override
  public void disabledPeriodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Drives Station is Connected
    if (alliance.isPresent()) {
      // Alliance Has Changed
      if (alliance.get() != alliance_) {
        alliance_ = alliance.get();
        // Update Driver Perspective
        SwerveDrivetrain.getInstance()
            .setDriverPerspective(
                alliance_ == Alliance.Red
                    ? SwerveDrivetrain.getInstance().RED_ALLIANCE_HEADING
                    : SwerveDrivetrain.getInstance().BLUE_ALLIANCE_HEADING);
      }
    }
  }

  @Override
  public void autonomousInit() {
    Elastic.selectTab("Auto");
    Auto auto = AutoManager.getInstance().getSelectedAuto();
    CommandScheduler.getInstance().schedule(auto);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    AutoManager.getInstance().removeDisplayedAuto();
  }

  @Override
  public void teleopInit() {
    ProxyServer.syncMatchData();
    SwerveDrivetrain.getInstance().restoreDefaultDriveMode();
    CommandScheduler.getInstance().cancelAll();
    Elastic.selectTab("Teleop");
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
