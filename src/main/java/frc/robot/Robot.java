// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.mw_lib.proxy_server.ProxyServer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective

  public Robot() {
    // Record metadata

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Setup Logging
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    OI.configureBindings();
    ProxyServer.configureServer();

    // Ensure subsystems are created
    Swerve.getInstance(); // ensure swerve is created
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
    if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(SwerveConstants.SWERVE_SIMULATION);
    }
  }

  @Override
  public void simulationPeriodic() {
    if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
      SimulatedArena.getInstance().simulationPeriodic();
      Logger.recordOutput(
          "FieldSimulation/RobotPosition",
          SwerveConstants.SWERVE_SIMULATION.getSimulatedDriveTrainPose());
      Logger.recordOutput(
          "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
      Logger.recordOutput(
          "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
  }
}
