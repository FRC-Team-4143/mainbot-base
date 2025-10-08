// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureTarget.Targets;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);

  public static void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Toggle field centric driving / robot centric driving
    driver_controller_.rightStick().onTrue(Swerve.getInstance().toggleFieldCentric());

    // Move to scoring positions
    driver_controller_.a().whileTrue(Commands.runOnce(() -> Superstructure.getInstance().requestMove(Targets.CORAL_INTAKE)));
    driver_controller_.x().whileTrue(Commands.runOnce(() -> Superstructure.getInstance().requestMove(Targets.L2)));
    driver_controller_.b().whileTrue(Commands.runOnce(() -> Superstructure.getInstance().requestMove(Targets.L3)));
    driver_controller_.y().whileTrue(
      new ConditionalCommand(
        Commands.runOnce(() -> Superstructure.getInstance().requestMove(Targets.L4)), 
        Commands.runOnce(() -> Superstructure.getInstance().requestMove(Targets.BARGE)), 
        () -> (Shooter.getInstance().getTargetGamePiece() == GamePiece.CORAL)));

    // Toggle target game piece for shooter
    driver_controller_.leftBumper().onTrue(Shooter.getInstance().toggleTargetGamePiece());

    // Intake coral
    driver_controller_.rightBumper().whileTrue(Commands.startEnd(() -> {
      Intake.getInstance().setWantedState(IntakeStates.PICK_UP);
      Shooter.getInstance().setWantedState(ShooterStates.GRABBING_CORAL);
    }, 
    () -> {
      Intake.getInstance().setWantedState(IntakeStates.IDLE);
      Shooter.getInstance().setWantedState(ShooterStates.IDLE);
    }).until(Shooter.getInstance()::hasCoral));

    // Shoot game piece
    driver_controller_.rightTrigger().whileTrue(Commands.startEnd(() -> Shooter.getInstance().shootGamePiece(), () -> Shooter.getInstance().stop()));
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
}
