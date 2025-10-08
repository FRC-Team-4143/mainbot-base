package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;

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

  // Simulation Object
  public static IntakeSimulation INTAKE_SIMULATOR;

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
  public final int ROLLER_ID = getIntConstant("roller", "id");
  public final double INTAKE_IN_SPEED = getDoubleConstant("roller", "in_speed");
  public final double INTAKE_OUT_SPEED = getDoubleConstant("roller", "out_speed");

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
