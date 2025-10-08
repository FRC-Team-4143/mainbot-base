// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

import java.util.function.BooleanSupplier;

public final class Constants {

  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final BooleanSupplier IS_REAL = () -> CURRENT_MODE == Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM
  }

  public static enum GamePiece {
    CORAL,
    ALGAE
  }

  public static final Color CORAL_COLOR = new Color(255, 255, 255);
  public static final Color ALGAE_COLOR = new Color(0, 255, 255);

  public static final double CONTROLLER_DEADBAND = 0.05;
}
