package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  public static class ElevatorConfig {
    public int leader_id_ = 0; // ID for the leader motor
    public int follower_id_ = 0; // ID for the follower motor
    public TalonFXConfiguration leader_config_ =
        new TalonFXConfiguration(); // Configuration for the leader motor
    public TalonFXConfiguration follower_config_ =
        new TalonFXConfiguration(); // Configuration for the follower motor
  }

  @AutoLog
  public static class ElevatorIOInputs {
    public double leader_position_ = 0.0; // Position in meters
    public double leader_velocity_ = 0.0; // Velocity in meters per second
    public double leader_applied_voltage_ = 0.0;
    public double leader_current_ = 0.0;
    public double leader_temp_ = 0.0; // Temperature in degrees Fahrenheit

    public double follower_position_ = 0.0; // Position in meters
    public double follower_velocity_ = 0.0; // Velocity in meters per second
    public double follower_applied_voltage_ = 0.0;
    public double follower_current_ = 0.0;
    public double follower_temp_ = 0.0; // Temperature in degrees Fahrenheit

    public double position_ = 0.0; // Position in meters
    public double velocity_ = 0.0; // Velocity in meters per second
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the elevator to a target position.
   *
   * @param position The target position in meters.
   */
  public default void setTargetPosition(double position) {}

  /** Zeroes the elevator position. */
  public default void tarePosition() {}

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public default void updateGains(Slot0Configs gains){}
}
