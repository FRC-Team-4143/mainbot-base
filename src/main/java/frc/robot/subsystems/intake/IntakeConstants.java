package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MWConstants;

public class IntakeConstants extends MWConstants {

  public enum IntakeStates {
    DEPLOY,
    PURGE,
    CLIMB_STAGE,
    PICK_UP,
    IDLE,
    TUNING
  }

  // General Intake Constants
  public final double INTAKE_WIDTH = 0.438150; // meters
  public final double INTAKE_LENGTH_EXTENDED = 0.281353; // meters
  public final double INTAKE_MASS = Units.lbsToKilograms(10); // kg
  public final double INTAKE_OFF_SET_Y = -Units.inchesToMeters(getDoubleConstant("offset_y"));
  public final double STATOR_CURRENT_LIMIT = getDoubleConstant("stator_current_limit");

  // TOF Constants
  public final int TIME_OF_FLIGHT_ID = getIntConstant("tof", "id");
  public final double TOF_CORAL_DISTANCE = Units.inchesToMeters(getDoubleConstant("tof", "coral_distance")) * 1000;

  // Intake Motor Constants
  public final int INTAKE_ID = getIntConstant("intake", "id");
  public final double INTAKE_IN_SPEED = getDoubleConstant("intake", "in_speed");
  public final double INTAKE_OUT_SPEED = getDoubleConstant("intake", "out_speed");

  // Pivot Motor Constants
  public final int PIVOT_ID = getIntConstant("pivot", "id");
  public final double PIVOT_MECH_RATIO = getDoubleConstant("pivot", "mech_ratio");
  public final Slot0Configs PIVOT_GAINS =
      new Slot0Configs()
          .withKP(getDoubleConstant("pivot", "kp"))
          .withKD(getDoubleConstant("pivot", "kd"));
  public final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration();

  public final Rotation2d PIVOT_OFFSET = Rotation2d.fromDegrees(35);
  public final Rotation2d PIVOT_DEPLOYED_ANGLE = Rotation2d.fromDegrees(-35);
  public final Rotation2d PIVOT_STATION_ANGLE = Rotation2d.fromDegrees(59);
  public final Rotation2d PIVOT_CLIMB_ANGLE = Rotation2d.fromDegrees(0);


  public IntakeConstants() {
    PIVOT_CONFIG.Slot0 = PIVOT_GAINS;
    PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    PIVOT_CONFIG.Feedback.SensorToMechanismRatio = PIVOT_MECH_RATIO;
    PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    PIVOT_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
  }
}
