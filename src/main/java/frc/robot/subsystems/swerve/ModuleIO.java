// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Annotations.Log;

public interface ModuleIO {
  public static class ModuleIOInputs {
    @Log.File public boolean driveConnected = false;
    @Log.File public double drivePositionRad = 0.0;
    @Log.File public double driveVelocityRadPerSec = 0.0;
    @Log.File public double driveAppliedVolts = 0.0;
    @Log.File public double driveCurrentAmps = 0.0;

    @Log.File public boolean turnConnected = false;
    @Log.File public boolean turnEncoderConnected = false;
    @Log.File public Rotation2d turnAbsolutePosition = new Rotation2d();
    @Log.File public Rotation2d turnPosition = new Rotation2d();
    @Log.File public double turnVelocityRadPerSec = 0.0;
    @Log.File public double turnAppliedVolts = 0.0;
    @Log.File public double turnCurrentAmps = 0.0;

    @Log.File public double[] odometryTimestamps = new double[] {};
    @Log.File public double[] odometryDrivePositionsRad = new double[] {};
    @Log.File public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
