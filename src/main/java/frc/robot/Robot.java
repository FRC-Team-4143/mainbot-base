// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.mw_lib.logging.GitLogger;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;

public class Robot extends TimedRobot {

  private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective

  public Robot() {
    // Record git metadata
    GitLogger.logGitData();

    // Setup Logging
    setupDataReceiversAndLogging();

    // Configure External Interfaces
    OI.configureBindings();
    ProxyServer.configureServer();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    // Call the scheduler so that commands work for buttons
    CommandScheduler.getInstance().run();

    // updates data from chassis proxy server
    ProxyServer.updateData();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {

      if (alliance.get() != alliance_) {
        alliance_ = alliance.get();
        Swerve.getInstance()
            .setOperatorForwardDirection(
                alliance_ == Alliance.Blue
                    ? SwerveConstants.OperatorPerspective.BLUE_ALLIANCE
                    : SwerveConstants.OperatorPerspective.RED_ALLIANCE);
      }
    }
  }

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

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    DogLog.log(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    DogLog.log(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  /** Set up data receivers and logging destinations */
  private void setupDataReceiversAndLogging() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        DogLog.setOptions(new DogLogOptions().withCaptureNt(true).withCaptureDs(true));
        DogLog.setPdh(new PowerDistribution());
        break;

      case SIM:
        DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true));
        break;
    }
    DogLog.setEnabled(true);
  }
}
