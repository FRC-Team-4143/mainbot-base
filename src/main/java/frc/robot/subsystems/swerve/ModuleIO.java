package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public boolean drive_connected_ = false;
    public double drive_position_ = 0.0; // Position in radians
    public double drive_velocity_ = 0.0; // Velocity in radians per second
    public double drive_applied_volts_ = 0.0;
    public double drive_current_ = 0.0;

    public boolean turn_connected_ = false;
    public Rotation2d turn_absolute_position_ = new Rotation2d();
    public Rotation2d turn_position_ = new Rotation2d();
    public double turn_velocity_ = 0.0; // Velocity in radians per second
    public double turn_applied_volts_ = 0.0;
    public double turn_current_ = 0.0;

    public double[] odometry_timestamps_ = new double[] {};
    public double[] odometry_drive_positions_ = new double[] {};
    public Rotation2d[] odometry_turn_positions_ = new Rotation2d[] {};
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
