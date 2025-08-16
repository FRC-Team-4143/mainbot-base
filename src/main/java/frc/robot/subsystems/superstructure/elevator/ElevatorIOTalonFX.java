package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leader_motor_;
  private final TalonFX follower_motor_;

  private final MotionMagicVoltage control_request_ = new MotionMagicVoltage(0.0);

  public ElevatorIOTalonFX(ElevatorConfig elevator_config) {
    // Initialize the TalonFX motors
    leader_motor_ = new TalonFX(elevator_config.leader_id_);
    follower_motor_ = new TalonFX(elevator_config.follower_id_);

    // Apply the configurations to the motors
    leader_motor_.getConfigurator().apply(elevator_config.leader_config_);
    follower_motor_.getConfigurator().apply(elevator_config.follower_config_);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update leader motor inputs
    inputs.leader_position_ = leader_motor_.getPosition().getValueAsDouble();
    inputs.leader_velocity_ = leader_motor_.getVelocity().getValueAsDouble();
    inputs.leader_applied_voltage_ = leader_motor_.getMotorVoltage().getValueAsDouble();
    inputs.leader_current_ = leader_motor_.getTorqueCurrent().getValueAsDouble();
    inputs.leader_temp_ = leader_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update follower motor inputs
    inputs.follower_position_ = follower_motor_.getPosition().getValueAsDouble();
    inputs.follower_velocity_ = follower_motor_.getVelocity().getValueAsDouble();
    inputs.follower_applied_voltage_ = follower_motor_.getMotorVoltage().getValueAsDouble();
    inputs.follower_current_ = follower_motor_.getTorqueCurrent().getValueAsDouble();
    inputs.follower_temp_ = follower_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update additional inputs
    inputs.position_ = inputs.leader_position_ * ElevatorConstants.ROTATIONS_TO_TRANSLATION;
    inputs.velocity_ = inputs.leader_velocity_ * ElevatorConstants.ROTATIONS_TO_TRANSLATION;
  }

  /**
   * Sets the elevator to a target position.
   *
   * @param position The target position in meters.
   */
  public void setTargetPosition(double position) {
    leader_motor_.setControl(
        control_request_.withPosition(position / ElevatorConstants.ROTATIONS_TO_TRANSLATION));
    follower_motor_.setControl(new StrictFollower(leader_motor_.getDeviceID()));
  }

  /** Zeroes the elevator position. */
  public void tarePosition() {
    leader_motor_.setPosition(0.0);
    follower_motor_.setPosition(0.0);
  }

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public void updateGains(Slot0Configs gains) {
    leader_motor_.getConfigurator().apply(gains);
  }
}
