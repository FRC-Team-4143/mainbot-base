// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Optional;

import frc.robot.commands.ExerciseExtend;
import frc.robot.commands.ExerciseTrunnion;
import frc.robot.commands.ExerciseTurret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public abstract class OI {

  // Sets up both controllers
  private static CommandXboxController driver_controller_ = new CommandXboxController(0);
  private static TalonFX turret = new TalonFX(22);
  private static TalonFX trunnion = new TalonFX(21);
  private static TalonFX extend = new TalonFX(20);

  public static void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
    turret.setNeutralMode(NeutralModeValue.Brake);
    trunnion.setNeutralMode(NeutralModeValue.Brake);
    extend.setNeutralMode(NeutralModeValue.Brake);
    driver_controller_.rightStick().onTrue(Swerve.getInstance().toggleFieldCentric());
    // driver_controller_.a().whileTrue(Commands.startEnd(() -> Superstructure.getInstance().requestMove(Targets.L3), () -> Superstructure.getInstance().requestMove(Targets.CORAL_INTAKE)));
  
    //driver_controller_.a().onTrue(Commands.run(() -> ElevatorSubsystem.getInstance().requestMove(0.0)));
    //driver_controller_.b().onTrue(Commands.run(() -> ElevatorSubsystem.getInstance().requestMove(1.0)));
    driver_controller_.x().whileTrue(Commands.startEnd(() -> turret.set(.2),
                                                       () -> turret.set(0)));
    driver_controller_.y().whileTrue(Commands.startEnd(() -> turret.set(-.2),
                                                       () -> turret.set(0)));
    driver_controller_.a().whileTrue(Commands.startEnd(() -> trunnion.set(.2),
                                                       () -> trunnion.set(0)));
    driver_controller_.b().whileTrue(Commands.startEnd(() -> trunnion.set(-.2),
                                                       () -> trunnion.set(0)));
    driver_controller_.leftBumper().whileTrue(Commands.startEnd(() -> extend.set(.6),
                                                                () -> extend.set(0)));
    driver_controller_.rightBumper().whileTrue(Commands.startEnd(() -> extend.set(-.6),
                                                                 () -> extend.set(0)));
    driver_controller_.leftTrigger().toggleOnTrue(new ExerciseTrunnion());
    driver_controller_.rightTrigger().toggleOnTrue(new ExerciseTurret());
    driver_controller_.start().toggleOnTrue(new ExerciseExtend());
    SmartDashboard.putData("SeedFieldCentric", Commands.runOnce(Swerve.getInstance()::seedFieldCentric));
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
