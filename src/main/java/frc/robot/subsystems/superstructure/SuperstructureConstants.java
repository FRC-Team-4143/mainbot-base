package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.mw_lib.subsystem.MWConstants;

public class SuperstructureConstants extends MWConstants {
  // State enum for the subsystem
  // Current system states for the superstructure
  public enum SuperstructureStates {
    AT_TARGET,
    UNSAFE_MOVE,
    SAFE_MOVE,
    RESCUE,
  }

  // Configurations for Elevator I/O
  public final int ELEVATOR_LEADER_ID = getIntConstant("elevator", "leader_id");
  public final TalonFXConfiguration ELEVATOR_LEADER_CONFIG;
  public final int ELEVATOR_FOLLOWER_ID = getIntConstant("elevator", "follower_id");
  public final TalonFXConfiguration ELEVATOR_FOLLOWER_CONFIG;

  // Configurations for Arm I/O
  public final int ARM_MOTOR_ID = getIntConstant("arm", "motor_id");
  public final TalonFXConfiguration ARM_MOTOR_CONFIG;
  public final int ARM_ENCODER_ID = getIntConstant("arm", "encoder_id");

  // Motion Ratios
  public final double ROTATIONS_TO_TRANSLATION = getDoubleConstant("elevator", "motion_ratio");
  public final double ARM_RATIO = getDoubleConstant("arm", "motion_ratio");

  // Arm and elevator target tolerances in SI units
  public final double ARM_TOLERANCE = getDoubleConstant("arm", "tolerance"); // rad
  public final double ELEV_TOLERANCE = getDoubleConstant("elevator", "tolerance"); // m

  // Meachanism limits
  public final double ELEVATOR_MIN_HEIGHT = getDoubleConstant("elevator", "min_height"); // m
  public final double ELEVATOR_MAX_HEIGHT = getDoubleConstant("elevator", "max_height"); // m
  public final double ARM_MIN_ANGLE = getDoubleConstant("arm", "min_angle"); // rad
  public final double ARM_MAX_ANGLE = getDoubleConstant("arm", "max_angle"); // rad

  // Simulation constants
  public final double CARRIAGE_MASS = 0.0; // kg
  public final double ARM_MOI = 0.0; // kg m^2
  public final double ARM_LENGTH = 0.0; // meters

  public SuperstructureConstants() {

    // Configure leader motor settings
    ELEVATOR_LEADER_CONFIG = new TalonFXConfiguration();
    ELEVATOR_LEADER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ELEVATOR_LEADER_CONFIG.CurrentLimits.StatorCurrentLimit =
        getDoubleConstant("elevator", "stator_limit");
    ELEVATOR_LEADER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ELEVATOR_LEADER_CONFIG.Slot0 =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("elevator", "kp"))
            .withKI(LOADER.getDoubleValue("elevator", "ki"))
            .withKD(LOADER.getDoubleValue("elevator", "ki"))
            .withKS(LOADER.getDoubleValue("elevator", "ks"))
            .withKV(LOADER.getDoubleValue("elevator", "kv"))
            .withKA(LOADER.getDoubleValue("elevator", "ka"))
            .withKG(LOADER.getDoubleValue("elevator", "kg"))
            .withGravityType(GravityTypeValue.Elevator_Static);
    ELEVATOR_LEADER_CONFIG.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(300.0)
            .withMotionMagicAcceleration(400.0)
            .withMotionMagicJerk(1000.0);

    // Configure follower motor settings
    ELEVATOR_FOLLOWER_CONFIG = new TalonFXConfiguration();
    ELEVATOR_FOLLOWER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ELEVATOR_FOLLOWER_CONFIG.CurrentLimits.StatorCurrentLimit =
        getDoubleConstant("elevator", "stator_limit");
    ELEVATOR_FOLLOWER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    ARM_MOTOR_CONFIG = new TalonFXConfiguration();
  }
}
