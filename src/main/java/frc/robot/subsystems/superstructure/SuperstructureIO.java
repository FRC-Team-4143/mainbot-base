package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.mw_lib.subsystem.SubsystemIO;

public abstract class SuperstructureIO extends SubsystemIO<SuperstructureConstants> {

  SuperstructureIO(SuperstructureConstants constants) {
    super(constants);
  }

  // Leader Elevator Motor
  /** Position in meters */
  public double current_leader_position = 0.0;

  /** Velocity in meters per second */
  public double current_leader_velocity = 0.0;

  /** Applied voltage */
  public double leader_applied_voltage = 0.0;

  /** Current in amperes */
  public double leader_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double leader_temp = 0.0;

  /** Target position i meters */
  public double target_elevator_position = 0.0;

  // Follower Elevator Motor
  /** Position in meters */
  public double current_follower_position = 0.0;

  /** Velocity in meters per second */
  public double current_follower_velocity = 0.0;

  /** Applied voltage */
  public double follower_applied_voltage = 0.0;

  /** Current in amperes */
  public double follower_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double follower_temp = 0.0;

  // Elevator Mechanism
  /** Position in meters */
  public double current_elevator_position = 0.0;

  /** Velocity in meters per second */
  public double current_elevator_velocity = 0.0;

  // Arm Mechanism
  /** Position in radians */
  public double current_arm_position = 0.0;

  /** Velocity in radians per second */
  public double current_arm_velocity = 0.0;

  /** Applied voltage */
  public double arm_applied_voltage = 0.0;

  /* Current in amperes */
  public double arm_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double arm_temp = 0.0;

  /** Target Position in radians */
  public double target_arm_position = 0.0;

  /** Zeroes the elevator and arm position. */
  public void tarePosition() {}

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public void updateElevatorGains(Slot0Configs gains) {}

  /**
   * Updates the gains for the arm.
   *
   * @param gains The new gains to apply.
   */
  public void updateArmGains(Slot0Configs gains) {}
}
