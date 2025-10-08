package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MWConstants;

public class ShooterConstants extends MWConstants {

  public enum ShooterStates {
    SHOOTING_CORAL,
    SHOOTING_ALGAE,
    GRABBING_CORAL,
    GRABBING_ALGAE,
    HOLDING_ALGAE,
    IDLE
  }

  // Intake Motor Constants
  public final int ROLLER_ID = getIntConstant("roller", "id");
  public final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();

  // TOF Constants
  public final int TIME_OF_FLIGHT_ID = getIntConstant("tof", "id");
  public final double TOF_CORAL_DISTANCE = Units.inchesToMeters(getDoubleConstant("tof", "coral_distance")) * 1000;

  // Current Sensing
  public final double ALGAE_VELOCITY_THRESHOLD = 0.5;

  public ShooterConstants() {
    ROLLER_CONFIG.CurrentLimits.StatorCurrentLimit = getDoubleConstant("roller", "stator_limit");
    ROLLER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    ROLLER_CONFIG.MotorOutput.Inverted = getBoolConstant( "roller", "ccw_positive") ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
  }
}
