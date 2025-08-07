// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveDrivetrain.DriveMode;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);
  private static CommandXboxController operator_controller_ = new CommandXboxController(1);

  private static BooleanSupplier pov_is_present_ = () -> getDriverJoystickPOV().isPresent();
  private static Trigger driver_pov_active_ = new Trigger(pov_is_present_);
  public static BooleanSupplier use_vision =
      () -> SmartDashboard.getBoolean("Vision/Use Vision Features", true);
  public static IntakePreference intake_preference = IntakePreference.GROUND;

  public enum IntakePreference {
    STATION,
    GROUND
  }

  public static void configureBindings() {
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

    /*
     *
     * Smart Dashboard Bindings
     *
     */
    // Set Wheel Offsets
    SmartDashboard.putData(
        "Commands/Set Wheel Offsets",
        Commands.runOnce(() -> SwerveDrivetrain.getInstance().tareEverything())
            .ignoringDisable(true));
    // Seed Field Centric Forward Direction
    SmartDashboard.putData(
        "Commands/Seed Field Centric",
        SwerveDrivetrain.getInstance().seedFieldRelativeCommand().ignoringDisable(true));
    SmartDashboard.putData(
        "Commands/Disturb Pose",
        Commands.runOnce(() -> PoseEstimator.getInstance().disturbPose()).ignoringDisable(true));
    SmartDashboard.putBoolean("Vision/Use Vision Features", true);

    /*
     *
     * Driver Controller Bindings
     *
     */

    // Crawl
    driver_pov_active_.whileTrue(
        Commands.startEnd(
            () -> SwerveDrivetrain.getInstance().setDriveMode(DriveMode.CRAWL),
            () -> SwerveDrivetrain.getInstance().restoreDefaultDriveMode()));

    /*
     *
     * Operator Controller Bindings
     *
     */
    operator_controller_
        .start()
        .onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Vision/Use Vision Features", true))
                .ignoringDisable(true));
    operator_controller_
        .back()
        .onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Vision/Use Vision Features", false))
                .ignoringDisable(true));
  }

  /**
   * @return driver controller left joystick x axis scaled quadratically
   */
  public static double getDriverJoystickLeftX() {
    double val = driver_controller_.getLeftX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  public static void toggleIntakePreference() {
    if (intake_preference == IntakePreference.GROUND) {
      intake_preference = IntakePreference.STATION;
    } else {
      intake_preference = IntakePreference.GROUND;
    }
  }

  public static boolean preferStationIntake() {
    return intake_preference == IntakePreference.STATION;
  }

  /**
   * @return driver controller left joystick y axis scaled quadratically
   */
  public static double getDriverJoystickLeftY() {
    double val = driver_controller_.getLeftY();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  /**
   * @return driver controller right joystick x axis scaled quadratically
   */
  public static double getDriverJoystickRightX() {
    double val = driver_controller_.getRightX();
    double output = val * val;
    output = Math.copySign(output, val);
    return val;
  }

  /**
   * @return driver controller joystick pov angle in degs. empty if nothing is pressed
   */
  public static Optional<Rotation2d> getDriverJoystickPOV() {
    int pov = driver_controller_.getHID().getPOV();
    return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
  }

  public static Command setRumble(double duration) {
    return Commands.startEnd(
            () -> driver_controller_.setRumble(RumbleType.kBothRumble, 1),
            () -> driver_controller_.setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration);
  }

  /*
   *
   * The OI methods below are used for the TalonFX Tuner Bindings.
   * These should not be used in teleop robot control.
   *
   */

  /**
   * @return event trigger bound to driver controller A button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickAButtonTrigger() {
    return driver_controller_.a();
  }

  /**
   * @return event trigger bound to driver controller B button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickBButtonTrigger() {
    return driver_controller_.b();
  }

  /**
   * @return event trigger bound to driver controller Y button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickYButtonTrigger() {
    return driver_controller_.y();
  }

  /**
   * @return event trigger bound to driver controller X button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getDriverJoystickXButtonTrigger() {
    return driver_controller_.x();
  }

  /**
   * @return event trigger bound to operator controller A button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickAButtonTrigger() {
    return operator_controller_.a();
  }

  /**
   * @return event trigger bound to driver controller B button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickBButtonTrigger() {
    return operator_controller_.b();
  }

  /**
   * @return event trigger bound to driver controller X button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickYButtonTrigger() {
    return operator_controller_.y();
  }

  /**
   * @return event trigger bound to driver controller Y button
   * @implNote DO NOT USE FOR TELEOP CONTROL
   */
  public static Trigger getOperatorJoystickXButtonTrigger() {
    return operator_controller_.x();
  }
}
