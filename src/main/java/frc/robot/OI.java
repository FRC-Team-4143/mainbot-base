// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);

  public static void configureBindings() {
    SmartDashboard.putData(
        "Reset Simulation Field", resetSimulationFieldCommand().ignoringDisable(true));

    driver_controller_.leftStick().onTrue(Swerve.getInstance().toggleFieldCentric());
  }

  /**
   * @return driver controller left joystick x axis scaled quadratically
   */
  public static double getDriverJoystickLeftX() {
    return driver_controller_.getLeftX();
  }

  /**
   * @return driver controller left joystick y axis scaled quadratically
   */
  public static double getDriverJoystickLeftY() {
    return driver_controller_.getLeftY();
  }

  /**
   * @return driver controller right joystick x axis scaled quadratically
   */
  public static double getDriverJoystickRightX() {
    return driver_controller_.getRightX();
  }

  /**
   * @return driver controller joystick pov angle in degs. empty if nothing is pressed
   */
  public static Optional<Rotation2d> getDriverJoystickPOV() {
    int pov = driver_controller_.getHID().getPOV();
    return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
  }

  /*
   * Resets the simulation field to the starting position
   */
  public static void resetSimulationField() {
    Robot.swerve_simulation_.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /*
   * @return Command to reset the simulation field to the starting position
   */
  private static Command resetSimulationFieldCommand() {
    return Commands.runOnce(() -> resetSimulationField());
  }
}
