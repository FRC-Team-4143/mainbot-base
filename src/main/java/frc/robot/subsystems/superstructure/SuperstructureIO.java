package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.mw_lib.subsystem.SubsystemIO;

public class SuperstructureIO implements SubsystemIO {

  public static class ElevatorConfig {
    public int leader_id_ = 0; // ID for the leader motor
    public int follower_id_ = 0; // ID for the follower motor
    public TalonFXConfiguration leader_config_ =
        new TalonFXConfiguration(); // Configuration for the leader motor
    public TalonFXConfiguration follower_config_ =
        new TalonFXConfiguration(); // Configuration for the follower motor
  }

  public static class ArmConfig {
    public int motor_id_ = 0; // ID for the arm motor
    public TalonFXConfiguration motor_config_ =
        new TalonFXConfiguration(); // Configuration for the arm motor
  }

  // Leader Elevator Motor
  public double current_leader_position = 0.0; // Position in meters
  public double current_leader_velocity = 0.0; // Velocity in meters per second
  public double leader_applied_voltage = 0.0;
  public double leader_current = 0.0;
  public double leader_temp = 0.0; // Temperature in degrees Fahrenheit

  public double target_elevator_position = 0.0; // Target position in meters

  // Follower Elevator Motor
  public double current_follower_position = 0.0; // Position in meters
  public double current_follower_velocity = 0.0; // Velocity in meters per second
  public double follower_applied_voltage = 0.0;
  public double follower_current = 0.0;
  public double follower_temp = 0.0; // Temperature in degrees Fahrenheit

  // Elevator Mechanism
  public double current_elevator_position = 0.0; // Position in meters
  public double current_elevator_velocity = 0.0; // Velocity in meters per second

  // Arm Mechanism
  public double current_arm_position = 0.0; // Position in radians
  public double current_arm_velocity = 0.0; // Velocity in radians per second
  public double arm_applied_voltage = 0.0;
  public double arm_current = 0.0;

  public double target_arm_position = 0.0; // Target position in radians

  /** Zeroes the elevator position. */
  public void tarePosition() {}

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public void updateElevatorGains(Slot0Configs gains) {}
}
