package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.mw_lib.util.ConstantsLoader;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorConfig;

public class ElevatorConstants {

    // ConstantsLoader instance for loading configuration values
    private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

    public static final ElevatorConfig ELEVATOR_CONFIG = new ElevatorConfig();

    public static final NeutralModeValue NEUTRAL_MODE =
        NeutralModeValue.Brake; // Default neutral mode for the elevator motors
    public static final double STATOR_CURRENT_LIMIT = 80.0; // Stator current limit in Amps
    // Units.inchesToMeters(Sprocket Circumference * Math.PI) / gearbox ratio * rigging
    public static final double ROTATIONS_TO_TRANSLATION = Units.inchesToMeters(1.751 * Math.PI)
    / LOADER.getDoubleValue("elevator", "ELEVATOR_GEAR_RATIO")
    * 2; // Conversion factor from motor rotations to meters for the elevator

  static {
    ELEVATOR_CONFIG.leader_id_ = 21; // ID for the leader motor
    ELEVATOR_CONFIG.follower_id_ = 22; // ID for the follower motor

    // Configure leader motor settings
    ELEVATOR_CONFIG.leader_config_ = new TalonFXConfiguration();
    ELEVATOR_CONFIG.leader_config_.MotorOutput.NeutralMode = NEUTRAL_MODE;
    ELEVATOR_CONFIG.leader_config_.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    ELEVATOR_CONFIG.leader_config_.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ELEVATOR_CONFIG.leader_config_.Slot0 =
        new Slot0Configs()
            .withKP(LOADER.getDoubleValue("elevator", "CONTROLLER_P"))
            .withKI(LOADER.getDoubleValue("elevator", "CONTROLLER_I"))
            .withKD(LOADER.getDoubleValue("elevator", "CONTROLLER_D"))
            .withKS(LOADER.getDoubleValue("elevator", "CONTROLLER_S"))
            .withKV(LOADER.getDoubleValue("elevator", "CONTROLLER_V"))
            .withKA(LOADER.getDoubleValue("elevator", "CONTROLLER_A"))
            .withKG(LOADER.getDoubleValue("elevator", "CONTROLLER_G"))
            .withGravityType(GravityTypeValue.Elevator_Static);
    ELEVATOR_CONFIG.leader_config_.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(300.0)
            .withMotionMagicAcceleration(400.0)
            .withMotionMagicJerk(1000.0);

    // Configure follower motor settings
    ELEVATOR_CONFIG.follower_config_ = new TalonFXConfiguration();
    ELEVATOR_CONFIG.follower_config_.MotorOutput.NeutralMode = NEUTRAL_MODE;
    ELEVATOR_CONFIG.follower_config_.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    ELEVATOR_CONFIG.follower_config_.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }
}
