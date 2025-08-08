// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Annotations.Log;

public interface GyroIO {
  public static class GyroIOInputs {
    @Log.NT public boolean connected = false;
    @Log.NT public Rotation2d yawPosition = new Rotation2d();
    @Log.NT public double yawVelocityRadPerSec = 0.0;
    @Log.NT public double[] odometryYawTimestamps = new double[] {};
    @Log.NT public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
