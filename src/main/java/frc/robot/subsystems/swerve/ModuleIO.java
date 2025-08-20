// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean drive_connected_ = false;
    public double drive_position_ = 0.0;
    public double drive_velocity_ = 0.0;
    public double drive_applied_volts_ = 0.0;
    public double drive_current_ = 0.0;

    public boolean turn_connected_ = false;
    public Rotation2d turn_position_ = new Rotation2d();
    public double turn_velocity_ = 0.0;
    public double turn_applied_volts_ = 0.0;
    public double turn_current_ = 0.0;

    public double[] odometry_timestamps_ = new double[] {};
    public double[] odometry_drive_positions_ = new double[] {};
    public Rotation2d[] odometry_turn_positions_ = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  default void setTurnPosition(Rotation2d rotation) {}
}
