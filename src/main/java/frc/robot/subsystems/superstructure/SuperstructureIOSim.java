package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SuperstructureIOSim extends SuperstructureIO {

  private final TalonFX elevator_motor_;
  private final TalonFX arm_motor_;
  private final DCMotor elevator_motor_sim_;
  private final DCMotor arm_motor_sim_;
  private final ElevatorSim elevator_sim_;
  private final SingleJointedArmSim arm_sim_;

  private final MotionMagicVoltage elevator_control_request_ = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage arm_control_request_ = new MotionMagicVoltage(0.0);

  public SuperstructureIOSim(SuperstructureConstants constants) {
    super(constants);

    // Initialize the TalonFX motors
    elevator_motor_ = new TalonFX(CONSTANTS.ELEVATOR_LEADER_ID);
    arm_motor_ = new TalonFX(CONSTANTS.ARM_MOTOR_ID);

    // Apply the configurations to the motors
    elevator_motor_.getConfigurator().apply(CONSTANTS.ELEVATOR_LEADER_CONFIG);
    arm_motor_.getConfigurator().apply(CONSTANTS.ARM_MOTOR_CONFIG);

    // Initialize the simulation objects
    elevator_motor_sim_ = DCMotor.getKrakenX60(2);
    arm_motor_sim_ = DCMotor.getKrakenX60(1);

    elevator_sim_ =
        new ElevatorSim(
            elevator_motor_sim_,
            1.0,
            CONSTANTS.CARRIAGE_MASS,
            CONSTANTS.ROTATIONS_TO_TRANSLATION,
            CONSTANTS.ELEVATOR_MIN_HEIGHT,
            CONSTANTS.ELEVATOR_MAX_HEIGHT,
            true,
            CONSTANTS.ELEVATOR_MIN_HEIGHT);

    arm_sim_ =
        new SingleJointedArmSim(
            arm_motor_sim_,
            1.0 / CONSTANTS.ARM_RATIO, // Gear ratio is inverse of sensor-to-mechanism ratio
            CONSTANTS.ARM_MOI,
            CONSTANTS.ARM_LENGTH,
            CONSTANTS.ARM_MIN_ANGLE,
            CONSTANTS.ARM_MAX_ANGLE,
            true,
            0);

    // Apply the simulation inversions
    elevator_motor_.getSimState().Orientation = CONSTANTS.ELEVATOR_LEADER_CONFIG.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
    arm_motor_.getSimState().Orientation = CONSTANTS.ARM_MOTOR_CONFIG.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
  }

  /** Updates the set of loggable inputs. */
  public void readInputs(double timestamp) {
    // Update elevator simulation
    elevator_motor_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    elevator_sim_.setInput(elevator_motor_.getSimState().getMotorVoltage());
    elevator_sim_.update(0.02);

    // Update arm simulation
    arm_motor_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    arm_sim_.setInput(arm_motor_.getSimState().getMotorVoltage());
    arm_sim_.update(0.02);

    // Simulate the battery voltage based on current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            elevator_sim_.getCurrentDrawAmps() + arm_sim_.getCurrentDrawAmps()));

    // Update simulation states with physics simulation results BEFORE next control cycle
    // This ensures the PID controller gets proper feedback from the physics simulation
    elevator_motor_
        .getSimState()
        .setRawRotorPosition(
            elevator_sim_.getPositionMeters() / CONSTANTS.ROTATIONS_TO_TRANSLATION);
    elevator_motor_
        .getSimState()
        .setRotorVelocity(
            elevator_sim_.getVelocityMetersPerSecond() / CONSTANTS.ROTATIONS_TO_TRANSLATION);

    // Apply the correct conversion for TalonFX simulation with SensorToMechanismRatio
    // The simulation state expects a value that, when processed by the ratio, gives the desired
    // reading
    arm_motor_
        .getSimState()
        .setRawRotorPosition(Units.radiansToRotations(arm_sim_.getAngleRads() * CONSTANTS.ARM_RATIO));
    arm_motor_
        .getSimState()
        .setRotorVelocity(Units.radiansToRotations(arm_sim_.getVelocityRadPerSec() * CONSTANTS.ARM_RATIO));

    // Update leader motor inputs
    current_leader_position = elevator_motor_.getPosition().getValueAsDouble();
    current_leader_velocity = elevator_motor_.getVelocity().getValueAsDouble();
    leader_applied_voltage = elevator_motor_.getMotorVoltage().getValueAsDouble();
    leader_current = elevator_motor_.getTorqueCurrent().getValueAsDouble();
    leader_temp = elevator_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update follower motor inputs
    current_follower_position = elevator_motor_.getPosition().getValueAsDouble();
    current_follower_velocity = elevator_motor_.getVelocity().getValueAsDouble();
    follower_applied_voltage = elevator_motor_.getMotorVoltage().getValueAsDouble();
    follower_current = elevator_motor_.getTorqueCurrent().getValueAsDouble();
    follower_temp = elevator_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update additional inputs
    current_elevator_position = current_leader_position * CONSTANTS.ROTATIONS_TO_TRANSLATION;
    current_elevator_velocity = current_leader_velocity * CONSTANTS.ROTATIONS_TO_TRANSLATION;

    // Update arm motor inputs
    current_arm_position = Units.rotationsToRadians(arm_motor_.getPosition().getValueAsDouble());
    current_arm_velocity = Units.rotationsToRadians(arm_motor_.getVelocity().getValueAsDouble());
    arm_applied_voltage = arm_motor_.getMotorVoltage().getValueAsDouble();
    arm_current = arm_motor_.getTorqueCurrent().getValueAsDouble();
    arm_temp = arm_motor_.getDeviceTemp().getValue().in(Fahrenheit);
    arm_encoder_position = arm_motor_.getPosition().getValueAsDouble();
  }

  /** Writes the desired outputs to the motors. */
  public void writeOutputs(double timestamp) {
    // Set the control request for the leader motor
    elevator_control_request_.withPosition(
        target_elevator_position / CONSTANTS.ROTATIONS_TO_TRANSLATION);
    elevator_motor_.setControl(elevator_control_request_);

    // Set the arm motor control request
    arm_motor_.setControl(
        arm_control_request_.withPosition(Units.radiansToRotations(target_arm_position)));
  }

  /** Zeroes the elevator position. */
  public void tarePosition() {
    elevator_motor_.setPosition(0.0);
    arm_motor_.setPosition(0.0);
  }

  public void updateElevatorGains(Slot0Configs gains) {
    DataLogManager.log("Updating Elevator Gains: " + gains.toString());
    elevator_motor_.getConfigurator().apply(gains);
    Slot0Configs current_gains = new Slot0Configs();
    elevator_motor_.getConfigurator().refresh(current_gains);
    DataLogManager.log("Updated Elevator Gains: " + current_gains.toString());
  }

  public void updateArmGains(Slot0Configs gains) {
    DataLogManager.log("Updating Arm Gains: " + gains.toString());
    arm_motor_.getConfigurator().apply(gains);
    Slot0Configs current_gains = new Slot0Configs();
    arm_motor_.getConfigurator().refresh(current_gains);
    DataLogManager.log("Updated Arm Gains: " + current_gains.toString());
  }
}
