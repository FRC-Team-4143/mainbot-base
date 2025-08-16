package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.mw_lib.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {

  // Pigeon 2 simulation model with some drift
  private final GyroSimulation gyro_sim_;

  GyroIOSim(GyroSimulation gyro_sim) {
    gyro_sim_ = gyro_sim;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected_ = true;
    inputs.yaw_position_ = gyro_sim_.getGyroReading();
    inputs.yaw_velocity_ = gyro_sim_.getMeasuredAngularVelocity().in(RadiansPerSecond);
    inputs.odometry_yaw_positions_ = gyro_sim_.getCachedGyroReadings();
    inputs.odometry_yaw_timestamps_ = PhoenixUtil.getSimulationOdometryTimeStamps();
  }
}
