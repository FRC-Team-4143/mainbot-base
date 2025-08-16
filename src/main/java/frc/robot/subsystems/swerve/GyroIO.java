package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected_ = false;
    public Rotation2d yaw_position_ = new Rotation2d();
    public double yaw_velocity_ = 0.0;
    public double[] odometry_yaw_timestamps_ = new double[] {};
    public Rotation2d[] odometry_yaw_positions_ = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
