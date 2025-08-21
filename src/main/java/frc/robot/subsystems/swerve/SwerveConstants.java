package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.module.ModuleType;

public class SwerveConstants {
  // CAN bus name and CAN ID for the pigeon2
  public static final int PIGEON2_ID = 0;
  public static final String PIGEON2_CANBUS_NAME = "CANivore";

  // CAN bus names for each of the swerve modules
  public static final String MODULE_CANBUS_NAME = "CANivore";

  // Determine the CAN bus update frequency for odometry
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(MODULE_CANBUS_NAME).isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
  public static final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
  public static final double DRIVE_FRICTION_VOLTAGE = 0.2;
  public static final double STEER_FRICTION_VOLTAGE = 0.2;

  public static final double ROBOT_MASS_KG =
      Units.lbsToKilograms(140.0); // Mass of the robot in pounds
  public static final double BUMPER_WIDTH_METERS =
      Units.inchesToMeters(34.75); // Width of the bumpers in meters (y axis : left -> right)
  public static final double BUMPER_LENGTH_METERS =
      Units.inchesToMeters(34.75); // Length of the bumpers in meters (x axis : front -> back)
  public static final double WHEEL_COF =
      1.916; // Coefficient of friction for the wheels VEX Grip V2

  // Forward Reference rotation constants
  public enum OperatorPerspective {
    BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
    RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

    private OperatorPerspective(Rotation2d heading) {
      this.heading = heading;
    }

    public final Rotation2d heading;
  }

  // The steer motor uses MotionMagicVoltage control
  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs().withKP(80).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // When using closed-loop control, the drive motor uses VelocityVoltage
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(0.5).withKI(0).withKD(0).withKS(0.18).withKV(0.117).withKA(0);

  // Control Constants for the swerve modules
  public static final double SLIP_CURRENT_AMPS = 50;
  public static final double SPEED_AT_12V_MPS = 5.0;
  public static final double COUPLE_RATIO = 3.5;
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.8);
  public static final double MAX_TRANSLATION_RATE = 5.0;
  public static final double MAX_ANGULAR_RATE = 10.0;

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      CONSTANT_CREATOR =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              // .withDriveMotorGearRatio(kDriveGearRatio) Unique to Module Type
              // .withSteerMotorGearRatio(kSteerGearRatio) Unique to Module Type
              .withCouplingGearRatio(COUPLE_RATIO)
              .withWheelRadius(WHEEL_RADIUS_METERS)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
              .withSlipCurrent(SLIP_CURRENT_AMPS)
              .withSpeedAt12Volts(SPEED_AT_12V_MPS)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              // .withFeedbackSource(kSteerFeedbackType) Not set for Analog Encoder
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(new TalonFXConfiguration())
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  // Front Left Module Constants
  private static final ModuleType FL_MODULE_TYPE = ModuleType.getModuleType("MK4N-L2+");
  public static final Translation2d FL_MODULE_TRANSLATION =
      new Translation2d(Units.inchesToMeters(11.4), Units.inchesToMeters(11.4));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FL_MODULE_CONSTANTS =
          CONSTANT_CREATOR
              .createModuleConstants(
                  2,
                  1,
                  0,
                  0,
                  FL_MODULE_TRANSLATION.getX(),
                  FL_MODULE_TRANSLATION.getY(),
                  false,
                  FL_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(FL_MODULE_TYPE.driveRatio)
              .withSteerMotorGearRatio(FL_MODULE_TYPE.steerRatio);

  // Front Right Module Constants
  private static final ModuleType FR_MODULE_TYPE = ModuleType.getModuleType("MK4I-L2+");
  public static final Translation2d FR_MODULE_TRANSLATION =
      new Translation2d(Units.inchesToMeters(11.4), Units.inchesToMeters(-11.4));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FR_MODULE_CONSTANTS =
          CONSTANT_CREATOR
              .createModuleConstants(
                  4,
                  3,
                  1,
                  0,
                  FR_MODULE_TRANSLATION.getX(),
                  FR_MODULE_TRANSLATION.getY(),
                  false,
                  FR_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(FR_MODULE_TYPE.driveRatio)
              .withSteerMotorGearRatio(FR_MODULE_TYPE.steerRatio);

  // Back Left Module Constants
  private static final ModuleType BL_MODULE_TYPE = ModuleType.getModuleType("MK4N-L2+");
  public static final Translation2d BL_MODULE_TRANSLATION =
      new Translation2d(Units.inchesToMeters(-11.4), Units.inchesToMeters(11.4));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BL_MODULE_CONSTANTS =
          CONSTANT_CREATOR
              .createModuleConstants(
                  6,
                  5,
                  2,
                  0,
                  BL_MODULE_TRANSLATION.getX(),
                  BL_MODULE_TRANSLATION.getY(),
                  false,
                  BL_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(BL_MODULE_TYPE.driveRatio)
              .withSteerMotorGearRatio(BL_MODULE_TYPE.steerRatio);

  // Back Right Module Constants
  private static final ModuleType BR_MODULE_TYPE = ModuleType.getModuleType("MK4I-L2+");
  public static final Translation2d BR_MODULE_TRANSLATION =
      new Translation2d(Units.inchesToMeters(-11.4), Units.inchesToMeters(-11.4));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BR_MODULE_CONSTANTS =
          CONSTANT_CREATOR
              .createModuleConstants(
                  8,
                  7,
                  3,
                  0,
                  BR_MODULE_TRANSLATION.getX(),
                  BR_MODULE_TRANSLATION.getY(),
                  false,
                  BR_MODULE_TYPE.steerInverted,
                  false)
              .withDriveMotorGearRatio(BR_MODULE_TYPE.driveRatio)
              .withSteerMotorGearRatio(BR_MODULE_TYPE.steerRatio);

  // Choreo Constants
  public static final boolean FLIP_TRAJECTORY_ON_RED = true;
  public static final double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
  public static final double CHOREO_X_CONTROLLER_KP = 0.0;
  public static final double CHOREO_X_CONTROLLER_KI = 0.0;
  public static final double CHOREO_X_CONTROLLER_KD = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KP = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KI = 0.0;
  public static final double CHOREO_Y_CONTROLLER_KD = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KP = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KI = 0.0;
  public static final double CHOREO_THETA_CONTROLLER_KD = 0.0;

  // Tractor Beam Constants
  public static final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
  public static final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
  public static final double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
  public static final double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
  public static final double TRACTOR_BEAM_CONTROLLER_KD = 0.0;
}
