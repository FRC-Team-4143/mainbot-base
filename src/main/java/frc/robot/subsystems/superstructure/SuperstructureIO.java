package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;

import dev.doglog.DogLog;
import frc.mw_lib.subsystem.SubsystemIO;

public abstract class SuperstructureIO extends SubsystemIO<SuperstructureConstants> {

  SuperstructureIO(SuperstructureConstants constants) {
    super(constants);
  }

  // Leader Elevator Motor
  /** Position in meters */
  public double leader_current_position = 0.0;

  /** Velocity in meters per second */
  public double leader_current_velocity = 0.0;

  /** Applied voltage */
  public double leader_applied_voltage = 0.0;

  /** Current in amperes */
  public double leader_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double leader_temp = 0.0;

  /** Target position in meters */
  public double elevator_target_position = 0.0;

  // Follower Elevator Motor
  /** Position in meters */
  public double follower_current_position = 0.0;

  /** Velocity in meters per second */
  public double follower_current_velocity = 0.0;

  /** Applied voltage */
  public double follower_applied_voltage = 0.0;

  /** Current in amperes */
  public double follower_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double follower_temp = 0.0;

  // Elevator Mechanism
  /** Position in meters */
  public double elevator_current_position = 0.0;

  /** Velocity in meters per second */
  public double elevator_current_velocity = 0.0;

  // Arm Mechanism
  /** Position in radians */
  public double arm_current_position = 0.0;

  /** Velocity in radians per second */
  public double arm_current_velocity = 0.0;

  /** Applied voltage */
  public double arm_applied_voltage = 0.0;

  /* Current in amperes */
  public double arm_current = 0.0;

  /** Temperature in degrees Fahrenheit */
  public double arm_temp = 0.0;

  /** Target Position in radians */
  public double arm_target_position = 0.0;

  /** Arm Encoder position in Rotation */
  public double arm_encoder_position = 0.0;

  /** Logs data to DogLog. */
  @Override
  public void logData(){
    DogLog.log(getSubsystemKey() + "Elevator/Leader/Position", leader_current_position);
    DogLog.log(getSubsystemKey() + "Elevator/Leader/Velocity", leader_current_velocity);
    DogLog.log(getSubsystemKey() + "Elevator/Leader/AppliedVoltage", leader_applied_voltage);
    DogLog.log(getSubsystemKey() + "Elevator/Leader/Current", leader_current);
    DogLog.log(getSubsystemKey() + "Elevator/Leader/Temperature", leader_temp);
    DogLog.log(getSubsystemKey() + "Elevator/Follower/Position", follower_current_position);
    DogLog.log(getSubsystemKey() + "Elevator/Follower/Velocity", follower_current_velocity);
    DogLog.log(getSubsystemKey() + "Elevator/Follower/AppliedVoltage", follower_applied_voltage);
    DogLog.log(getSubsystemKey() + "Elevator/Follower/Current", follower_current);
    DogLog.log(getSubsystemKey() + "Elevator/Follower/Temperature", follower_temp);
    DogLog.log(getSubsystemKey() + "Elevator/Position/Current", elevator_current_position);
    DogLog.log(getSubsystemKey() + "Elevator/Position/Target", elevator_target_position);
    DogLog.log(getSubsystemKey() + "Elevator/Velocity", elevator_current_velocity);
    DogLog.log(getSubsystemKey() + "Arm/Position/Current", arm_current_position);
    DogLog.log(getSubsystemKey() + "Arm/Position/Target", arm_target_position);
    DogLog.log(getSubsystemKey() + "Arm/Velocity", arm_current_velocity);
    DogLog.log(getSubsystemKey() + "Arm/AppliedVoltage", arm_applied_voltage);
    DogLog.log(getSubsystemKey() + "Arm/Current", arm_current);
    DogLog.log(getSubsystemKey() + "Arm/Temperature", arm_temp);
    DogLog.log(getSubsystemKey() + "Arm/EncoderPosition", arm_encoder_position);
  }

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
